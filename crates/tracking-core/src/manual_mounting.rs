//! Explicit manual fallback. Heading/bend-axis edits preserve the neutral pose;
//! physical XYZ trims intentionally change sensor-to-bone orientation.
use crate::{CoreError, Role, TrackingCore, wrap_angle};
use glam::Quat;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ManualMountValues {
    pub forward_degrees: f32,
    pub physical_degrees_xyz: [f32; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ManualMountAdjustment {
    pub role: Role,
    pub values: ManualMountValues,
    pub confirmed: bool,
    pub can_undo: bool,
}

/// Manual edits change only the right mount and the initial heading alignment.
/// Dynamic yaw (recovery, knee correction) is a separate left factor that
/// commutes with the paired edit, so it is never captured or restored here.
#[derive(Clone, Copy)]
pub(crate) struct ManualMountState {
    base_mount: Quat,
    base_alignment: f32,
    base_confirmed: bool,
    neutral: Quat,
    values: ManualMountValues,
    previous: Option<ManualRestore>,
}

#[derive(Clone, Copy)]
struct ManualRestore {
    values: ManualMountValues,
    confirmed: bool,
    mounting: Quat,
    alignment: f32,
}

impl ManualMountState {
    /// Standing space was rotated about +Y: stored alignments rotate with it.
    pub(crate) fn rebase_heading(&mut self, delta: f32) {
        self.base_alignment = wrap_angle(self.base_alignment + delta);
        if let Some(previous) = &mut self.previous {
            previous.alignment = wrap_angle(previous.alignment + delta);
        }
    }
}

fn validate(values: ManualMountValues) -> Result<(), CoreError> {
    if std::iter::once(values.forward_degrees)
        .chain(values.physical_degrees_xyz)
        .any(|angle| !angle.is_finite() || !(-180.0..=180.0).contains(&angle))
    {
        return Err(CoreError(
            "Manual angles must be finite and between -180° and 180°".into(),
        ));
    }
    Ok(())
}

fn adjusted(state: &ManualMountState) -> (Quat, f32) {
    let delta = state.values.forward_degrees.to_radians();
    // A'=Ry(d)A and M'=q0^-1 Ry(-d) q0 M: A'q0M'=Aq0M.
    // This changes the bend axis, not the captured upright orientation.
    let paired_mount = state.neutral.conjugate()
        * Quat::from_rotation_y(-delta)
        * state.neutral
        * state.base_mount;
    let [x, y, z] = state.values.physical_degrees_xyz.map(f32::to_radians);
    // Explicit local quaternion product Rx*Ry*Rz, applied on the RIGHT. This
    // physical trim is separate from the gravity-preserving paired yaw edit.
    let physical = Quat::from_rotation_x(x) * Quat::from_rotation_y(y) * Quat::from_rotation_z(z);
    (
        (paired_mount * physical).normalize(),
        wrap_angle(state.base_alignment + delta),
    )
}

impl TrackingCore {
    pub fn manual_mount_adjustments(&self) -> Vec<ManualMountAdjustment> {
        self.assignments
            .keys()
            .filter(|role| self.mounting.contains_key(role))
            .map(|role| {
                let state = self.manual_mounts.get(role);
                ManualMountAdjustment {
                    role: *role,
                    values: state.map_or_else(ManualMountValues::default, |state| state.values),
                    confirmed: self.functional_roles.contains(role) && self.direction_session_valid,
                    can_undo: state.is_some_and(|state| state.previous.is_some()),
                }
            })
            .collect()
    }

    pub fn set_manual_mount(
        &mut self,
        role: Role,
        values: ManualMountValues,
    ) -> Result<(), CoreError> {
        validate(values)?;
        if !self.calibrated {
            return Err(CoreError(
                "Capture neutral before manually adjusting tracker mounting".into(),
            ));
        }
        let mut state = if let Some(state) = self.manual_mounts.get(&role) {
            *state
        } else {
            let neutral = self
                .neutral_reference
                .as_ref()
                .and_then(|reference| reference.orientations.get(&role))
                .ok_or_else(|| CoreError("Selected tracker needs a neutral reference".into()))?;
            if self.assignments.get(&role) != Some(&neutral.0) {
                return Err(CoreError(
                    "Selected tracker assignment changed; capture neutral again".into(),
                ));
            }
            ManualMountState {
                base_mount: *self.mounting.get(&role).ok_or_else(|| {
                    CoreError("Selected tracker has no mounting reference".into())
                })?,
                base_alignment: self.heading_alignment.get(&role).copied().unwrap_or(0.),
                base_confirmed: self.functional_roles.contains(&role)
                    && self.direction_session_valid,
                neutral: neutral.1,
                values: ManualMountValues::default(),
                previous: None,
            }
        };
        if values == state.values {
            return Ok(());
        }
        state.previous = Some(ManualRestore {
            values: state.values,
            confirmed: self.functional_roles.contains(&role),
            mounting: self.mounting[&role],
            alignment: self.heading_alignment.get(&role).copied().unwrap_or(0.),
        });
        state.values = values;
        // Alignment changes by the forward delta only; dynamic yaw (recovery
        // or knee correction) stays in `yaw` and commutes with this edit.
        let (mount, alignment) = adjusted(&state);
        if !mount.is_finite() || !alignment.is_finite() {
            return Err(CoreError("Invalid manual mounting rotation".into()));
        }
        self.mounting.insert(role, mount);
        self.preview_motion.reset();
        self.heading_alignment.insert(role, alignment);
        self.functional_roles.remove(&role);
        self.manual_mounts.insert(role, state);
        self.knee_history.clear();
        self.reset_stance_layers_for(role);
        self.recovery = crate::RecoveryState::Idle;
        self.recovery_undo = None;
        self.calibration_reason = Some(
            "Manual mounting changed; test movement and explicitly confirm its direction".into(),
        );
        Ok(())
    }

    /// Explicit user assertion after checking actual movement, not a measurement
    /// or an automatic result. All six core roles are required for readiness.
    pub fn confirm_manual_mount(&mut self, role: Role) -> Result<(), CoreError> {
        if !self.calibrated
            || !self.mounting.contains_key(&role)
            || !self.assignments.contains_key(&role)
        {
            return Err(CoreError(
                "Selected tracker needs a current neutral calibration".into(),
            ));
        }
        let certified = self.functional_roles.contains(&role) && self.direction_session_valid;
        if !certified && !self.manual_mounts.contains_key(&role) {
            // Yaw on a never-certified, unedited role was never checked by any
            // gate; it must not become active through this confirmation. Yaw
            // kept through a manual edit was accrued while certified and is
            // part of the output the user just checked, so it is retained.
            self.yaw.insert(role, 0.);
        }
        self.functional_roles.insert(role);
        self.preview_motion.reset();
        self.reset_stance_layers_for(role);
        self.direction_session_valid = true;
        self.calibration_reason = Some(
            "Manual direction confirmed by user; verify movement before enabling output".into(),
        );
        Ok(())
    }

    pub fn undo_manual_mount(&mut self, role: Role) -> Result<(), CoreError> {
        let mut state = *self
            .manual_mounts
            .get(&role)
            .ok_or_else(|| CoreError("No manual adjustment to undo for this tracker".into()))?;
        let previous = state
            .previous
            .take()
            .ok_or_else(|| CoreError("No previous manual adjustment for this tracker".into()))?;
        state.values = previous.values;
        // Restore mount and alignment only; yaw accrued since the edit stays.
        self.mounting.insert(role, previous.mounting);
        self.preview_motion.reset();
        self.heading_alignment.insert(role, previous.alignment);
        if previous.confirmed {
            self.functional_roles.insert(role);
        } else {
            self.functional_roles.remove(&role);
        }
        self.manual_mounts.insert(role, state);
        self.knee_history.clear();
        self.reset_stance_layers_for(role);
        self.recovery = crate::RecoveryState::Idle;
        self.recovery_undo = None;
        Ok(())
    }

    /// Return this editor to the mounting that existed when its first edit began.
    pub fn reset_manual_mount(&mut self, role: Role) -> Result<(), CoreError> {
        let state = self
            .manual_mounts
            .remove(&role)
            .ok_or_else(|| CoreError("No manual adjustment to reset for this tracker".into()))?;
        self.mounting.insert(role, state.base_mount);
        self.preview_motion.reset();
        // Yaw accrued since the first edit is kept.
        self.heading_alignment.insert(role, state.base_alignment);
        if state.base_confirmed {
            self.functional_roles.insert(role);
        } else {
            self.functional_roles.remove(&role);
        }
        self.knee_history.clear();
        self.reset_stance_layers_for(role);
        self.recovery = crate::RecoveryState::Idle;
        self.recovery_undo = None;
        Ok(())
    }

    /// A manual edit, undo, reset or confirmation of a core role changes what
    /// the stance corrections were estimated against: the whole filter starts
    /// again from zero. Optional roles only interrupt contacts and leg IK.
    fn reset_stance_layers_for(&mut self, role: Role) {
        if Role::CORE.contains(&role) {
            self.reset_stance_layers();
        } else {
            self.interrupt_stance_layers();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;
    fn neutral_core() -> TrackingCore {
        let mut core = TrackingCore::new(crate::TrackingConfig::default()).unwrap();
        core.set_assignments(
            Role::CORE
                .iter()
                .map(|role| crate::RoleAssignment {
                    sensor_id: format!("manual-{role:?}"),
                    role: *role,
                })
                .collect(),
        )
        .unwrap();
        for step in 0..=100 {
            for role in Role::CORE {
                core.ingest(crate::SensorSample {
                    id: format!("manual-{role:?}"),
                    received_at_secs: f64::from(step) / 100.,
                    orientation_xyzw: Quat::IDENTITY.to_array(),
                    acceleration_mps2: Some([0.; 3]),
                    provenance: crate::SensorProvenance::Measured,
                })
                .unwrap();
            }
        }
        core.calibrate_neutral(1., &crate::Anchors::default(), true)
            .unwrap();
        core
    }
    #[test]
    fn manual_edits_require_confirmation_and_undo_restores_previous_certification_and_yaw() {
        let mut core = neutral_core();
        assert!(!core.direction_calibrated());
        for role in Role::CORE {
            core.confirm_manual_mount(role).unwrap();
        }
        assert!(core.direction_calibrated());
        core.yaw.insert(Role::LeftThigh, 0.1);
        let alignment_before = core.heading_alignment[&Role::LeftThigh];
        let before_edit = core.segment(Role::LeftThigh, 1.).rotation;
        core.set_manual_mount(
            Role::LeftThigh,
            ManualMountValues {
                forward_degrees: 90.,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(crate::angle(before_edit, core.segment(Role::LeftThigh, 1.).rotation) < 0.001);
        assert!(!core.direction_calibrated());
        // The edit changes alignment by exactly the forward delta; yaw stays.
        assert!(
            (core.heading_alignment[&Role::LeftThigh]
                - crate::wrap_angle(alignment_before + 90_f32.to_radians()))
            .abs()
                < 1e-6
        );
        assert_eq!(core.yaw[&Role::LeftThigh], 0.1);
        // Yaw accrued after the edit (e.g. a later correction) survives undo.
        core.yaw.insert(Role::LeftThigh, 0.25);
        core.undo_manual_mount(Role::LeftThigh).unwrap();
        assert!(core.direction_calibrated());
        assert_eq!(core.yaw[&Role::LeftThigh], 0.25);
        assert_eq!(core.heading_alignment[&Role::LeftThigh], alignment_before);
        core.set_manual_mount(
            Role::LeftThigh,
            ManualMountValues {
                forward_degrees: -45.,
                ..Default::default()
            },
        )
        .unwrap();
        core.confirm_manual_mount(Role::LeftThigh).unwrap();
        assert_eq!(core.yaw[&Role::LeftThigh], 0.25);
        core.yaw.insert(Role::LeftThigh, 0.3);
        core.reset_manual_mount(Role::LeftThigh).unwrap();
        assert!(core.direction_calibrated());
        assert_eq!(core.yaw[&Role::LeftThigh], 0.3);
        assert_eq!(core.heading_alignment[&Role::LeftThigh], alignment_before);
        // A never-certified, unedited role cannot activate unchecked yaw.
        core.clear_calibration();
        let mut core = neutral_core();
        core.yaw.insert(Role::LeftCalf, 0.2);
        core.confirm_manual_mount(Role::LeftCalf).unwrap();
        assert_eq!(core.yaw[&Role::LeftCalf], 0.);
        core.yaw.insert(Role::LeftCalf, 0.2);
        core.confirm_manual_mount(Role::LeftCalf).unwrap();
        assert_eq!(core.yaw[&Role::LeftCalf], 0.2, "re-confirm keeps yaw");
        core.clear_calibration();
        assert!(core.manual_mounts.is_empty());
        assert!(core.confirm_manual_mount(Role::LeftThigh).is_err());
    }
    #[test]
    fn invalid_manual_edit_is_atomic_and_yaw_preserves_existing_physical_tilt() {
        let mut core = neutral_core();
        let values = ManualMountValues {
            forward_degrees: 0.,
            physical_degrees_xyz: [20., 0., -15.],
        };
        core.set_manual_mount(Role::Hip, values).unwrap();
        let neutral_before = core.segment(Role::Hip, 1.).rotation;
        core.set_manual_mount(
            Role::Hip,
            ManualMountValues {
                forward_degrees: 90.,
                ..values
            },
        )
        .unwrap();
        assert!(crate::angle(neutral_before, core.segment(Role::Hip, 1.).rotation) < 0.001);
        let before = core.manual_mount_adjustments();
        assert!(
            core.set_manual_mount(
                Role::Hip,
                ManualMountValues {
                    forward_degrees: f32::NAN,
                    ..values
                }
            )
            .is_err()
        );
        assert_eq!(
            serde_json::to_value(before).unwrap(),
            serde_json::to_value(core.manual_mount_adjustments()).unwrap()
        );
    }
    #[test]
    fn paired_yaw_changes_bend_axis_without_changing_neutral_under_arbitrary_mount() {
        let neutral = Quat::from_rotation_y(1.4) * Quat::from_rotation_z(0.8);
        let base_alignment = -0.3;
        let reference = Quat::from_rotation_y(0.7);
        let base_mount = (Quat::from_rotation_y(base_alignment) * neutral).conjugate() * reference;
        let state = ManualMountState {
            base_mount,
            base_alignment,
            base_confirmed: false,
            neutral,
            values: ManualMountValues {
                forward_degrees: 90.,
                ..Default::default()
            },
            previous: None,
        };
        let (mount, alignment) = adjusted(&state);
        assert!(
            crate::angle(
                Quat::from_rotation_y(alignment) * neutral * mount,
                reference
            ) < 0.001
        );
        let tilted = Quat::from_rotation_x(0.4) * neutral;
        let before = Quat::from_rotation_y(base_alignment) * tilted * base_mount;
        let after = Quat::from_rotation_y(alignment) * tilted * mount;
        assert!((before * Vec3::Y).distance(after * Vec3::Y) > 0.3);
    }
    #[test]
    fn physical_trim_changes_standing_orientation_and_rejects_invalid_values() {
        let mut state = ManualMountState {
            base_mount: Quat::IDENTITY,
            base_alignment: 0.,
            base_confirmed: false,
            neutral: Quat::IDENTITY,
            values: ManualMountValues::default(),
            previous: None,
        };
        state.values.physical_degrees_xyz = [30., 0., 0.];
        let (mount, _) = adjusted(&state);
        assert!(crate::angle(mount, Quat::from_rotation_x(30_f32.to_radians())) < 0.001);
        for invalid in [f32::NAN, f32::INFINITY, 181., -181.] {
            assert!(
                validate(ManualMountValues {
                    forward_degrees: invalid,
                    ..Default::default()
                })
                .is_err()
            );
        }
    }
}
