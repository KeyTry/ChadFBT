//! Native application interface and local perspective preview. The model is the
//! sole runtime boundary; no socket, calibration, or solver state lives here.
use crate::{
    calibration_request::CalibrationRequestPhase,
    identification,
    model::{
        AppHandle, CalibrationMethod, CalibrationRequestKind, Command, ROLES, Snapshot,
        TrackerView, WorkerStatus,
    },
    output_mode::{OutputMode, OutputSink},
};
use eframe::egui::{self, Align, Color32, FontId, Layout, RichText, Sense, Stroke, Vec2};
use glam::{Quat, Vec3};
use std::{
    sync::mpsc,
    time::{Duration, Instant},
};
use tracking_core::Role;
use tracking_core::motion_calibration::{
    MotionCalibrationStatus, MotionGate, MotionReference, MotionSegmentStatus, MotionWarningKind,
};
use tray_icon::{
    TrayIcon, TrayIconBuilder, TrayIconEvent,
    menu::{Menu, MenuEvent, MenuItem, PredefinedMenuItem},
};

const BG: Color32 = Color32::from_rgb(12, 18, 27);
const PANEL: Color32 = Color32::from_rgb(19, 28, 40);
const CARD: Color32 = Color32::from_rgb(25, 37, 51);
const LINE: Color32 = Color32::from_rgb(42, 57, 73);
const TEXT: Color32 = Color32::from_rgb(229, 238, 246);
const MUTED: Color32 = Color32::from_rgb(145, 165, 184);
const TEAL: Color32 = Color32::from_rgb(55, 222, 190);
const AMBER: Color32 = Color32::from_rgb(248, 191, 103);

pub fn run(handle: AppHandle) -> eframe::Result {
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default()
            .with_title("ChadFBT | Independent body tracking")
            .with_inner_size([1180., 720.])
            .with_min_inner_size([940., 620.])
            .with_maximized(true),
        ..Default::default()
    };
    eframe::run_native(
        "ChadFBT",
        options,
        Box::new(move |cc| Ok(Box::new(ChadApp::new(cc, handle)))),
    )
}

#[derive(Clone, Copy)]
enum TrayAction {
    Show,
    Recover,
    Exit,
}

struct ChadApp {
    handle: AppHandle,
    page: usize,
    camera: Camera,
    show_axes: bool,
    show_scene: bool,
    height: f32,
    /// Edited locally but not yet committed to the worker.
    height_dirty: bool,
    tray: Option<TrayIcon>,
    tray_events: mpsc::Receiver<TrayAction>,
    quitting: bool,
    tray_error: Option<String>,
    neutral_asserted: bool,
    identification_clock: Instant,
    identification_role: String,
    identification_session: Option<identification::Session>,
    identification_notice: Option<String>,
    height_capture_asserted: bool,
    last_height_capture_id: u64,
    last_neutral_capture_id: u64,
    manual_role: String,
    manual_values: [f32; 4],
    manual_observed: Option<[f32; 4]>,
    /// Prediction lead being dragged; committed on release like the height.
    prediction_lead_ms: f32,
    prediction_lead_dirty: bool,
}

impl ChadApp {
    fn new(cc: &eframe::CreationContext<'_>, handle: AppHandle) -> Self {
        let mut style = (*cc.egui_ctx.style_of(egui::Theme::Dark)).clone();
        style.visuals = egui::Visuals::dark();
        style.visuals.panel_fill = BG;
        style.visuals.window_fill = PANEL;
        style.visuals.extreme_bg_color = BG;
        style.visuals.override_text_color = Some(TEXT);
        style.visuals.widgets.noninteractive.bg_stroke = Stroke::new(1., LINE);
        style.visuals.widgets.inactive.bg_fill = CARD;
        style.visuals.widgets.inactive.weak_bg_fill = CARD;
        style.visuals.widgets.inactive.bg_stroke = Stroke::new(1., LINE);
        style.visuals.widgets.hovered.bg_fill = Color32::from_rgb(38, 67, 78);
        style.visuals.widgets.hovered.weak_bg_fill = Color32::from_rgb(38, 67, 78);
        style.visuals.widgets.hovered.bg_stroke = Stroke::new(1., TEAL);
        style.visuals.widgets.active.bg_fill = Color32::from_rgb(31, 91, 86);
        style.visuals.selection.bg_fill = Color32::from_rgb(26, 99, 92);
        style.visuals.selection.stroke = Stroke::new(1., TEAL);
        style.spacing.item_spacing = egui::vec2(10., 9.);
        style.spacing.button_padding = egui::vec2(12., 8.);
        style
            .text_styles
            .insert(egui::TextStyle::Heading, FontId::proportional(24.));
        style
            .text_styles
            .insert(egui::TextStyle::Body, FontId::proportional(14.));
        style
            .text_styles
            .insert(egui::TextStyle::Small, FontId::proportional(12.));
        cc.egui_ctx.set_style_of(egui::Theme::Dark, style);
        cc.egui_ctx.set_theme(egui::Theme::Dark);
        let wake_context = cc.egui_ctx.clone();
        crate::single_instance::set_waker(std::sync::Arc::new(move || {
            wake_context.request_repaint()
        }));
        let (tx, tray_events) = mpsc::channel();
        let tray_result = create_tray(&cc.egui_ctx, tx);
        let (tray, tray_error) = match tray_result {
            Ok(tray) => (Some(tray), None),
            Err(error) => (None, Some(format!("System tray unavailable: {error}"))),
        };
        let initial = handle.snapshot();
        let height = initial.height_m;
        let prediction_lead_ms = initial.estimator_settings.prediction_lead_ms;
        Self {
            handle,
            page: 0,
            camera: Camera::default(),
            show_axes: true,
            show_scene: true,
            height: if height > 0.5 { height } else { 1.65 },
            height_dirty: false,
            tray,
            tray_events,
            quitting: false,
            tray_error,
            neutral_asserted: false,
            identification_clock: Instant::now(),
            identification_role: "chest".to_owned(),
            identification_session: None,
            identification_notice: None,
            height_capture_asserted: false,
            last_height_capture_id: 0,
            last_neutral_capture_id: 0,
            manual_role: String::new(),
            manual_values: [0.; 4],
            manual_observed: None,
            prediction_lead_ms,
            prediction_lead_dirty: false,
        }
    }

    fn quit(&mut self, ctx: &egui::Context) {
        self.quitting = true;
        self.handle.send(Command::Shutdown);
        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
    }

    fn header(&mut self, ui: &mut egui::Ui, s: &Snapshot) {
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("CHAD").size(28.).strong().color(TEXT));
                    ui.label(RichText::new("FBT").size(28.).strong().color(TEAL));
                    badge(ui, "RESEARCH BUILD", MUTED);
                });
                ui.label(
                    RichText::new("Independent body tracking. Your trackers, your movement.")
                        .small()
                        .color(MUTED),
                );
            });
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add_enabled(self.tray.is_some(), egui::Button::new("Hide to tray"))
                    .on_hover_text("Tracking continues. Click the tray icon to return.")
                    .clicked()
                {
                    ui.ctx()
                        .send_viewport_cmd(egui::ViewportCommand::Visible(false));
                }
                let online = s.trackers.iter().filter(|t| connected(t)).count();
                badge(
                    ui,
                    &format!("{online} TRACKERS CONNECTED"),
                    if online > 0 { TEAL } else { AMBER },
                );
                badge(
                    ui,
                    if calibration_active(s) {
                        "CAPTURE IN PROGRESS"
                    } else if s.direction_calibrated {
                        "CALIBRATED"
                    } else if s.calibrated && s.calibration_method == CalibrationMethod::Movement {
                        "MOVEMENT CALIBRATION NEEDED"
                    } else if s.calibrated {
                        "FORWARD BEND NEEDED"
                    } else {
                        "SETUP REQUIRED"
                    },
                    if s.direction_calibrated && !calibration_active(s) {
                        TEAL
                    } else {
                        AMBER
                    },
                );
            });
        });
    }

    fn sidebar(&mut self, ui: &mut egui::Ui, s: &Snapshot) {
        ui.horizontal(|ui| {
            for (index, title) in ["Setup", "Trackers", "Health"].iter().enumerate() {
                if ui.selectable_label(self.page == index, *title).clicked() {
                    self.page = index;
                }
            }
        });
        ui.separator();
        egui::ScrollArea::vertical()
            .id_salt("sidebar_scroll")
            .show(ui, |ui| match self.page {
                0 => self.setup(ui, s),
                1 => self.trackers(ui, s),
                _ => self.health(ui, s),
            });
    }

    fn setup(&mut self, ui: &mut egui::Ui, s: &Snapshot) {
        section(ui, "01", "Connect your trackers", |ui| {
            ui.label("Power on your WiFi trackers on this PC's local network.");
            let online = s.trackers.iter().filter(|t| connected(t)).count();
            ui.label(
                RichText::new(format!(
                    "{online} connected  /  {} discovered",
                    s.trackers.len()
                ))
                .color(if online > 0 { TEAL } else { AMBER }),
            );
            small(
                ui,
                "Close other tracker servers so ChadFBT can listen on UDP 6969. Existing tracker firmware stays installed.",
            );
            if online == 0 {
                hint(
                    ui,
                    "Waiting for tracker traffic. If discovery stays empty, launch ChadFBT normally from Windows and check its Private network firewall access.",
                    AMBER,
                );
            }
        });
        section(ui, "02", "Assign body locations", |ui| {
            small(
                ui,
                "For six trackers: chest, hip, both thighs and both calves. Match each physical tracker to its location.",
            );
            self.identification(ui, s);
            self.assignments(ui, s);
        });
        section(ui, "03", "Set your standing scale", |ui| {
            ui.label("Standing eye height");
            ui.horizontal(|ui| {
                let response = ui.add(
                    egui::DragValue::new(&mut self.height)
                        .range(0.8..=2.5)
                        .speed(0.005)
                        .fixed_decimals(3)
                        .suffix(" m"),
                );
                self.height_dirty |= response.changed();
                // Each SetHeight saves the profile and cancels a pending capture:
                // commit once on drag release or focus loss, not on every frame.
                if self.height_dirty
                    && (response.drag_stopped()
                        || response.lost_focus()
                        || (!response.dragged() && !response.has_focus()))
                {
                    self.height_dirty = false;
                    self.handle.send(Command::SetHeight(self.height));
                }
                ui.label(RichText::new(format!("{:.0} cm", self.height * 100.)).color(MUTED));
            });
            small(
                ui,
                "Measure floor to eyes while standing; this is lower than total body height. The stored value changes only when you edit it or explicitly capture headset height.",
            );
            if let Some(height) = s.hmd_standing_height_m {
                let difference = height - s.height_m;
                let tolerance = s.calibration_height_tolerance_m;
                ui.label(
                    RichText::new(format!("Live headset standing height: {height:.3} m")).strong(),
                );
                small(
                    ui,
                    &format!(
                        "Difference from saved eye height: {:+.1} cm · allowed ±{:.1} cm",
                        difference * 100.,
                        tolerance * 100.
                    ),
                );
                if tolerance > 0. && difference.abs() > tolerance {
                    hint(
                        ui,
                        "Headset height does not match the saved eye height. Stand upright, look forward, and check SteamVR's floor. If the saved value is your total height, explicitly capture eye height below.",
                        AMBER,
                    );
                }
            } else {
                small(
                    ui,
                    "Live headset standing height: unavailable (a fresh, valid headset pose is required).",
                );
            }
            ui.checkbox(
                &mut self.height_capture_asserted,
                "I will stand upright with the headset on before the countdown ends",
            );
            if ui
                .add_enabled(
                    self.height_capture_asserted
                        && s.hmd_standing_height_m.is_some()
                        && !calibration_active(s),
                    egui::Button::new("Use current headset height (3 s)"),
                )
                .clicked()
            {
                self.handle.send(Command::UseCurrentHeadsetHeight);
                self.height_capture_asserted = false;
            }
            small(
                ui,
                "After the countdown, your current headset height becomes the saved eye height. Check the floor level before capturing.",
            );
            let mut auto_height = s.auto_eye_height;
            if ui
                .checkbox(&mut auto_height, "Update eye height from the headset at each live neutral capture")
                .on_hover_text("When the averaged standing headset height differs from the saved eye height by more than 1.5 cm (and stays within the live height tolerance), it becomes the eye height before the capture; body dimensions are rescaled. Changes over 5 cm are announced.")
                .changed()
            {
                self.handle.send(Command::SetAutoEyeHeight(auto_height));
            }
            if s.calibration_request_kind == CalibrationRequestKind::StandingHeight {
                self.calibration_feedback(ui, s);
            }
            ui.horizontal_wrapped(|ui| {
                badge(
                    ui,
                    if s.hmd_connected {
                        "HMD LIVE"
                    } else {
                        "NO LIVE HMD"
                    },
                    if s.hmd_connected { TEAL } else { AMBER },
                );
                badge(
                    ui,
                    &format!(
                        "{} / 2 HANDS",
                        s.hands_connected.iter().filter(|x| **x).count()
                    ),
                    MUTED,
                );
            });
        });
        section(ui, "04", "Calibrate your pose", |ui| {
            ui.label("Stand upright with feet parallel under your hips and arms relaxed. Face your head and torso in the same direction and keep still.");
            small(ui, &s.mode_status);
            if s.desktop_session {
                hint(
                    ui,
                    "Desktop calibration (no worn headset, or Desktop only mode): choose a real-world direction and face it throughout capture. The floor arrow marks preview forward; the trackers cannot see your monitor. Nothing is sent. Outside Desktop only it is aligned to the headset automatically when you put one on.",
                    AMBER,
                );
                let mut motion = s.desktop_motion_enabled;
                if ui
                    .checkbox(&mut motion, "Estimate body movement (experimental)")
                    .changed()
                {
                    self.handle.send(Command::SetDesktopMotion(motion));
                }
                small(
                    ui,
                    "Estimates movement from leg motion and possible foot contact. It can drift or slip. Recalibrate neutral to return to the center.",
                );
                if !s.direction_calibrated {
                    small(
                        ui,
                        "Estimated movement starts after calibration (by movement, or neutral plus skiing pose).",
                    );
                } else if !s.desktop_motion_status.is_empty() {
                    small(ui, &s.desktop_motion_status);
                }
            }
            if !s.desktop_session {
                small(
                    ui,
                    "Choose a comfortable direction and face it with your headset and torso throughout capture. The headset supplies the live facing reference; the camera view does not change it.",
                );
            }
            let required = [
                "chest",
                "hip",
                "left_thigh",
                "right_thigh",
                "left_calf",
                "right_calf",
            ];
            let ready = required
                .iter()
                .filter(|role| {
                    s.trackers.iter().any(|tracker| {
                        tracker.role.as_deref() == Some(**role) && tracker.eligible_for_calibration
                    })
                })
                .count();
            let all_ready = ready == required.len();
            match s.calibration_method {
                CalibrationMethod::Movement => self.movement_calibration(ui, s, all_ready),
                CalibrationMethod::SkiPose => self.ski_pose_calibration(ui, s, all_ready),
            }
            ui.horizontal_wrapped(|ui| {
                if ui
                    .add_enabled(!calibration_active(s), egui::Button::new("Test sound"))
                    .clicked()
                {
                    self.handle.send(Command::TestCalibrationSound);
                }
                let mut mode = s.audio_mode;
                egui::ComboBox::from_id_salt("audio_mode")
                    .selected_text(mode.label())
                    .show_ui(ui, |ui| {
                        for option in crate::audio_cues::AudioMode::ALL {
                            ui.selectable_value(&mut mode, option, option.label());
                        }
                    });
                if mode != s.audio_mode {
                    self.handle.send(Command::SetAudioMode(mode));
                }
                small(
                    ui,
                    "Spoken prompts and tones use the Windows default audio device.",
                );
            });
            if !s.voice_status.is_empty()
                && s.audio_mode == crate::audio_cues::AudioMode::VoiceAndTones
            {
                small(ui, &s.voice_status);
            }
            if let Some(text) = floor_mismatch_text(s.floor_mismatch_m) {
                hint(ui, &text, AMBER);
            }
            if !s.alignment_status.is_empty() {
                hint(
                    ui,
                    &format!(
                        "Aligning the desktop calibration with the headset: {}",
                        s.alignment_status
                    ),
                    AMBER,
                );
            }
            if !all_ready {
                small(
                    ui,
                    &format!(
                        "{ready}/6 required locations ready: chest, hip, both thighs and both calves. Each needs a fresh measurement or explicit firmware-held compatibility eligibility; connection alone is insufficient."
                    ),
                );
            }
            if !s.desktop_session && !s.hmd_connected {
                hint(
                    ui,
                    "This mode calibrates with the headset: it needs a fresh, valid headset pose. Put the headset on, or choose Desktop only. A failed attempt leaves its exact reason beside the button.",
                    AMBER,
                );
            }
            if s.calibrated && ui.button("Clear calibration").clicked() {
                self.handle.send(Command::ClearCalibration);
            }
        });
        self.manual_mounting(ui, s);
        if ui.button("Save setup profile").clicked() {
            self.handle.send(Command::SaveProfile);
        }
        small(ui, "Body assignments and standing scale are saved locally.");
    }

    /// Primary direction calibration: neutral capture, then a bow and knee
    /// lifts (or squats) accumulated until every segment is ready.
    fn movement_calibration(&mut self, ui: &mut egui::Ui, s: &Snapshot, all_ready: bool) {
        egui::Frame::new()
            .fill(BG)
            .stroke(Stroke::new(1., LINE))
            .corner_radius(10)
            .inner_margin(12)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("Calibrate by movement").strong());
                    badge(ui, "EXPERIMENTAL", MUTED);
                });
                small(
                    ui,
                    "First stand upright and still for the neutral capture after a 3-second countdown.",
                );
                hint(ui, MOVEMENT_INSTRUCTIONS, TEAL);
                small(
                    ui,
                    if s.desktop_session {
                        "Desktop calibration: choose a real-world direction and face it for the neutral capture and the whole movement. Nothing is sent until it is aligned with a worn headset."
                    } else {
                        "The neutral capture needs the live headset. Its movement during the bow also helps find forward."
                    },
                );
                let label = if s.direction_calibrated {
                    "Recalibrate by movement (3 s)"
                } else {
                    "Calibrate by movement (3 s)"
                };
                if primary(ui, label, all_ready && !calibration_active(s)).clicked() {
                    self.handle.send(Command::CalibrateMotion);
                    self.neutral_asserted = false;
                }
                if s.motion_move_available {
                    let failed = s.calibration_request_kind == CalibrationRequestKind::MotionPose
                        && s.calibration_request.phase == CalibrationRequestPhase::TimedOut;
                    let text = if failed {
                        "Try movement again (keep neutral, 3 s)"
                    } else {
                        "Start at the movement step (keep current neutral, 3 s)"
                    };
                    if ui
                        .add_enabled(
                            all_ready && !calibration_active(s),
                            egui::Button::new(text),
                        )
                        .on_hover_text("Reuses the neutral pose captured in the last five minutes. Stand where you were and face the same direction.")
                        .clicked()
                    {
                        self.handle.send(Command::CalibrateMotionMove);
                    }
                }
                ui.label(RichText::new(limits_note(s.desktop_session)).small().color(MUTED))
                    .on_hover_text(limits_detail(s.desktop_session));
                if s.calibration_request_kind == CalibrationRequestKind::MotionPose {
                    self.calibration_feedback(ui, s);
                    motion_progress(ui, s);
                }
            });
        ui.add_space(4.);
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(
                    !calibration_active(s),
                    egui::Button::new("Use held skiing pose instead"),
                )
                .on_hover_text(
                    "Fallback: capture the neutral pose, then hold a skiing pose for a few seconds.",
                )
                .clicked()
            {
                self.handle
                    .send(Command::SetCalibrationMethod(CalibrationMethod::SkiPose));
            }
            small(ui, "Fallback if movement calibration does not work for you.");
        });
    }

    /// Fallback: the original two-step neutral plus held skiing pose flow.
    fn ski_pose_calibration(&mut self, ui: &mut egui::Ui, s: &Snapshot, all_ready: bool) {
        ui.horizontal_wrapped(|ui| {
            badge(ui, "FALLBACK · HELD SKIING POSE", AMBER);
            if ui
                .add_enabled(
                    !calibration_active(s),
                    egui::Button::new("Calibrate by movement instead"),
                )
                .clicked()
            {
                self.handle
                    .send(Command::SetCalibrationMethod(CalibrationMethod::Movement));
            }
        });
        ui.label(RichText::new("1. Capture a neutral pose").strong());
        let calibration = if s.calibrated {
            "Recalibrate neutral pose (3 s)"
        } else {
            "Calibrate neutral pose (3 s)"
        };
        if primary(ui, calibration, all_ready && !calibration_active(s)).clicked() {
            self.handle.send(Command::Calibrate);
            self.neutral_asserted = false;
        }
        small(
            ui,
            "Click, then stand upright and face forward. A three-second countdown precedes capture; sounds play when available. A brief stable-pose check ends in success or a visible reason. Cancel stops the request.",
        );
        if s.calibration_request_kind == CalibrationRequestKind::NeutralPose {
            self.calibration_feedback(ui, s);
        }
        ui.add_space(5.);
        ui.label(RichText::new("2. Capture a forward bend").strong());
        small(
            ui,
            "Keep your feet flat and parallel. Bend your knees forward, move your hips back, and lean forward without turning. Your pelvis can tilt less than your chest; the segments do not need matching angles. Keep facing the same direction as neutral.",
        );
        draw_forward_bend_guide(ui);
        small(
            ui,
            "After the 3-second countdown, you have up to 10 seconds to settle. Small sway is allowed. Hold your bend until the success tone or a final result.",
        );
        if primary(
            ui,
            "Calibrate forward bend (3 s)",
            s.calibrated && !calibration_active(s),
        )
        .clicked()
        {
            self.handle.send(Command::CalibrateForward);
        }
        small(
            ui,
            if s.direction_calibrated {
                "Forward direction configured. Test your movement; this experimental calibration still needs live validation."
            } else if s.calibrated {
                "Neutral captured · capture the forward bend, or configure the advanced manual fallback, before live output."
            } else {
                "Capture the neutral pose first. The forward bend helps determine tracker mounting; the arrow alone cannot do this."
            },
        );
        if s.calibration_request_kind == CalibrationRequestKind::ForwardPose {
            self.calibration_feedback(ui, s);
        }
    }

    fn manual_mounting(&mut self, ui: &mut egui::Ui, s: &Snapshot) {
        egui::CollapsingHeader::new("Advanced · manual mounting fallback")
            .id_salt("manual_mounting")
            .show(ui, |ui| {
                small(ui, "Use this fallback after neutral calibration if the forward-bend capture does not work. Adjust each assigned tracker, move it to check the result, then confirm its direction. This does not prove the pose or tracking accuracy.");
                if calibration_active(s) {
                    small(ui, "Manual adjustments are unavailable during capture.");
                    return;
                }
                if !s.calibrated || s.manual_mounts.is_empty() {
                    small(ui, "Capture a neutral pose with assigned trackers first.");
                    return;
                }
                if !s.manual_mounts.iter().any(|m| m.role == self.manual_role) {
                    self.manual_role = s.manual_mounts[0].role.clone();
                    self.manual_observed = None;
                }
                let old_role = self.manual_role.clone();
                egui::ComboBox::from_id_salt("manual_role")
                    .selected_text(body_role_label(&self.manual_role))
                    .show_ui(ui, |ui| {
                        for mount in &s.manual_mounts {
                            ui.selectable_value(&mut self.manual_role, mount.role.clone(), body_role_label(&mount.role));
                        }
                    });
                if old_role != self.manual_role { self.manual_observed = None; }
                let Some(mount) = s.manual_mounts.iter().find(|m| m.role == self.manual_role) else { return; };
                let observed = [mount.forward_degrees, mount.physical_degrees_xyz[0], mount.physical_degrees_xyz[1], mount.physical_degrees_xyz[2]];
                if self.manual_observed != Some(observed) {
                    self.manual_values = observed;
                    self.manual_observed = Some(observed);
                }
                let sensor = s.trackers.iter().find(|t| t.role.as_deref() == Some(self.manual_role.as_str()));
                if let Some(sensor) = sensor {
                    small(ui, &format!("Selected: {} · {}", body_role_label(&self.manual_role), short_tracker_id(&sensor.id)));
                    assignment_orientation_status(ui, sensor);
                    draw_raw_sensor_axes(ui, sensor);
                }
                ui.label(RichText::new("Bend direction · Y").strong());
                small(ui, "Changes which way this tracker bends while preserving its captured neutral pose. Start here.");
                let bend = ui.add(egui::Slider::new(&mut self.manual_values[0], -180.0..=180.0).suffix("°").fixed_decimals(1));
                let mut commit = bend.drag_stopped() || bend.lost_focus() || (bend.changed() && bend.clicked());
                ui.add_space(4.);
                ui.label(RichText::new("Physical mount trim · local X / Y / Z").strong());
                small(ui, "Drag an axis handle or use the sliders. The local trim uses Rx × Ry × Rz. Unlike bend direction, X and Z can change the captured standing tilt.");
                commit |= draw_manual_gizmo(ui, &mut self.manual_values[1..4]);
                for (index, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                    let response = ui.add(egui::Slider::new(&mut self.manual_values[index + 1], -180.0..=180.0).text(label).suffix("°").fixed_decimals(1));
                    commit |= response.drag_stopped() || response.lost_focus() || (response.changed() && response.clicked());
                }
                if ui.button("Apply adjustments").clicked() { commit = true; }
                if commit && self.manual_values != observed && self.manual_values.iter().all(|value| value.is_finite()) {
                    self.handle.send(Command::SetManualMount {
                        role: self.manual_role.clone(),
                        forward_degrees: self.manual_values[0],
                        physical_degrees_xyz: [self.manual_values[1], self.manual_values[2], self.manual_values[3]],
                    });
                }
                ui.horizontal_wrapped(|ui| {
                    if ui.add_enabled(mount.can_undo && !calibration_active(s), egui::Button::new("Undo adjustment")).clicked() {
                        self.handle.send(Command::UndoManualMount { role: self.manual_role.clone() });
                    }
                    if ui.add_enabled(!calibration_active(s) && (mount.can_undo || observed.iter().any(|angle| *angle != 0.)), egui::Button::new("Reset this tracker")).clicked() {
                        self.handle.send(Command::ResetManualMount { role: self.manual_role.clone() });
                    }
                });
                small(ui, if mount.confirmed { "Direction confirmed for this tracker." } else { "Direction not yet confirmed for this tracker." });
                if ui.add_enabled(!calibration_active(s), egui::Button::new("Confirm this tracker's direction")).clicked() {
                    if self.manual_values != observed && self.manual_values.iter().all(|value| value.is_finite()) {
                        self.handle.send(Command::SetManualMount {
                            role: self.manual_role.clone(),
                            forward_degrees: self.manual_values[0],
                            physical_degrees_xyz: [self.manual_values[1], self.manual_values[2], self.manual_values[3]],
                        });
                    }
                    self.handle.send(Command::ConfirmManualMount { role: self.manual_role.clone() });
                }
                if !s.manual_mount_message.is_empty() { hint(ui, &s.manual_mount_message, AMBER); }
                small(ui, "Confirm only after moving this tracker and checking that the corresponding body segment moves in the intended direction. Confirm every required tracker before live output.");
            });
        ui.add_space(8.);
    }

    fn calibration_feedback(&self, ui: &mut egui::Ui, snapshot: &Snapshot) {
        let request = &snapshot.calibration_request;
        if request.phase == CalibrationRequestPhase::Idle {
            return;
        }
        let action = match snapshot.calibration_request_kind {
            CalibrationRequestKind::StandingHeight => "Headset height",
            CalibrationRequestKind::NeutralPose => "Neutral pose",
            CalibrationRequestKind::ForwardPose => "Forward bend",
            CalibrationRequestKind::MotionPose => "Movement calibration",
        };
        match request.phase {
            CalibrationRequestPhase::Countdown => {
                ui.label(
                    RichText::new(format!(
                        "{action}: {:.0}…",
                        request.seconds_remaining.ceil()
                    ))
                    .size(25.)
                    .strong()
                    .color(TEAL),
                );
                hint(ui, &request.message, TEAL);
            }
            CalibrationRequestPhase::WaitingStable => {
                hint(ui, &request.message, AMBER);
                small(
                    ui,
                    &format!(
                        "Stable-input check ends in {:.1} s. Hold your pose and keep facing forward.",
                        request.seconds_remaining
                    ),
                );
            }
            CalibrationRequestPhase::Moving => {
                ui.label(
                    RichText::new(format!(
                        "Move now · {:.0} s left",
                        request.seconds_remaining.ceil()
                    ))
                    .size(22.)
                    .strong()
                    .color(TEAL),
                );
                hint(ui, &request.message, TEAL);
                small(
                    ui,
                    &format!(
                        "Moving for {:.1} s. Finishes by itself once every segment stays ready for half a second (after at least 3 s); then stand normally.",
                        snapshot.motion_elapsed_secs
                    ),
                );
            }
            CalibrationRequestPhase::Captured => {
                hint(ui, &request.message, TEAL);
                if snapshot.calibration_request_kind == CalibrationRequestKind::NeutralPose {
                    small(
                        ui,
                        if snapshot.direction_calibrated && snapshot.desktop_session {
                            "Neutral capture succeeded and the learned mounting is retained. It is aligned to the headset automatically when you put one on (not in Desktop only)."
                        } else if snapshot.direction_calibrated {
                            "Neutral capture succeeded and the learned mounting is retained. Test your movement."
                        } else if snapshot.desktop_session {
                            "Neutral capture succeeded. Capture the forward bend next; the result is aligned to the headset automatically when you put one on."
                        } else {
                            "Neutral capture succeeded. Capture the forward bend next."
                        },
                    );
                }
            }
            CalibrationRequestPhase::TimedOut | CalibrationRequestPhase::Cancelled => {
                hint(ui, &request.message, AMBER);
                small(
                    ui,
                    if snapshot.calibration_request_kind == CalibrationRequestKind::MotionPose
                        && snapshot.motion_move_available
                    {
                        "No delayed capture remains. Try the movement again (the neutral capture is kept) or use the held skiing pose instead."
                    } else {
                        "No delayed capture remains. Correct the reason, then start a new 3-second countdown."
                    },
                );
            }
            CalibrationRequestPhase::Idle => {}
        }
        if snapshot.calibrated
            && matches!(
                request.phase,
                CalibrationRequestPhase::Countdown
                    | CalibrationRequestPhase::WaitingStable
                    | CalibrationRequestPhase::TimedOut
                    | CalibrationRequestPhase::Cancelled
            )
        {
            small(
                ui,
                "Previous alignment is still stored; this request has not produced a new capture. Output pauses during a countdown or capture and otherwise follows the output mode.",
            );
        }
        if calibration_active(snapshot) && ui.button("Cancel capture").clicked() {
            self.handle.send(Command::CancelCalibration);
        }
    }

    fn identification(&mut self, ui: &mut egui::Ui, snapshot: &Snapshot) {
        egui::Frame::new().fill(BG).stroke(Stroke::new(1., LINE)).corner_radius(10).inner_margin(12).show(ui, |ui| {
            ui.label(RichText::new("Identify a tracker by movement").strong());
            small(ui, "Choose its body location. Keep the other trackers still, then gently rotate only this tracker a quarter-turn and back during the 3-second capture.");
            let has_session = self.identification_session.is_some();
            let role_label = body_role_label(&self.identification_role);
            ui.add_enabled_ui(!has_session, |ui| {
                egui::ComboBox::from_id_salt("identify_role").selected_text(role_label).width(ui.available_width()).show_ui(ui, |ui| {
                    for (id, label) in ROLES { ui.selectable_value(&mut self.identification_role, (*id).to_owned(), *label); }
                });
            });
            if let Some(notice) = &self.identification_notice { hint(ui, notice, TEAL); }
            let mut start = false;
            let mut clear = false;
            let mut assignment = None;
            if let Some(session) = &self.identification_session {
                match session.outcome() {
                    None => {
                        let remaining = (identification::WINDOW_SECONDS*(1.-f64::from(session.progress()))).max(0.);
                        activity_meter(ui, session.progress(), &format!("Rotate only {role_label} · {remaining:.1} s"), TEAL);
                        small(ui, "Slow, deliberate rotation works best. The app observes snapshot changes and can miss a fast back-and-forth between frames.");
                    }
                    Some(identification::Outcome::Candidate { tracker_id }) => {
                        let candidate = snapshot.trackers.iter().find(|tracker| &tracker.id == tracker_id);
                        let label = candidate.map_or("Unknown tracker", |tracker| tracker.label.as_str());
                        hint(ui, &format!("Movement match: {label}"), TEAL);
                        if let Some(current) = candidate.and_then(|tracker| tracker.role.as_deref()) {
                            small(ui, &format!("Currently assigned to {}.", body_role_label(current)));
                        }
                        if let Some(replaced) = snapshot.trackers.iter().find(|tracker| tracker.role.as_deref() == Some(self.identification_role.as_str()) && tracker.id != *tracker_id) {
                            small(ui, &format!("This will replace {} in the {role_label} role.", replaced.label));
                        }
                        let observations = identification_observations(snapshot);
                        let available = session.candidate_still_connected(&observations);
                        if primary(ui, &format!("Assign this tracker to {role_label}"), available).clicked() {
                            assignment = Some(tracker_id.clone());
                        }
                        if !available { hint(ui, "The candidate disconnected or its counter restarted. Reconnect it and retry; no assignment has been made.", AMBER); }
                        small(ui, "Assign only if this is the tracker you deliberately moved. Assignments can reset neutral calibration.");
                    }
                    Some(result) => {
                        let message = match result {
                            identification::Outcome::NoMovement => "No clear movement observed. Rotate only the intended tracker farther and retry.",
                            identification::Outcome::Ambiguous => "Multiple trackers moved, or the lead was unclear. Keep the others still and retry; nothing was assigned.",
                            identification::Outcome::CandidateDisconnected => "The moving tracker lost its connection during capture. Reconnect it and retry; nothing was assigned.",
                            identification::Outcome::Interrupted => "The device list, packet counter or observation timing changed. Retry a complete 3-second capture; nothing was assigned.",
                            identification::Outcome::Candidate { .. } => unreachable!(),
                        };
                        hint(ui, message, AMBER);
                    }
                }
                // Keep retry/cancel visible before the potentially long device list.
                ui.horizontal_wrapped(|ui| {
                    if session.outcome().is_some() && ui.button("Try again").clicked() { start = true; }
                    if ui.button(if session.outcome().is_none() {"Cancel capture"} else {"Choose another location"}).clicked() { clear = true; }
                });
                let activities = session.activities();
                let highest = activities.first().map_or(1., |activity| activity.observed_degrees_per_second.max(1.));
                for activity in activities {
                    let label = snapshot.trackers.iter().find(|tracker| tracker.id == activity.tracker_id).map_or_else(|| short_tracker_id(&activity.tracker_id), |tracker| tracker.label.clone());
                    let candidate = session.candidate_id() == Some(activity.tracker_id.as_str());
                    activity_meter(ui, (activity.observed_degrees_per_second/highest) as f32,
                        &format!("{label} · {:.0}°/s observed{}", activity.observed_degrees_per_second, if activity.connected {""} else {" · disconnected"}),
                        if candidate {TEAL} else {MUTED});
                }
                small(ui, "Observed angular activity is a movement comparison, not an IMU sampling rate or accuracy score. Firmware-held repetitions add no motion.");
            } else {
                if primary(ui, "Start 3-second identification", snapshot.trackers.iter().any(|tracker| connected(tracker) && !tracker.transport_invalidated)).clicked() { start = true; }
                small(ui, "A clear match is only a suggestion. You confirm the body assignment afterward.");
            }
            if clear { self.identification_session = None; self.identification_notice = None; }
            if start {
                self.identification_session = Some(identification::Session::start(self.identification_clock.elapsed().as_secs_f64(), &identification_observations(snapshot)));
                self.identification_notice = None;
            }
            if let Some(id) = assignment {
                self.handle.send(Command::Assign {id, role: Some(self.identification_role.clone())});
                self.identification_notice = Some(format!("Assignment requested for {role_label}. Choose the next body location when ready."));
                self.identification_session = None;
            }
        });
    }

    fn assignments(&self, ui: &mut egui::Ui, s: &Snapshot) {
        if s.trackers.is_empty() {
            small(ui, "Trackers appear here as they connect.");
            return;
        }
        for tracker in &s.trackers {
            let candidate = self
                .identification_session
                .as_ref()
                .and_then(|session| session.candidate_id())
                == Some(tracker.id.as_str());
            egui::Frame::new()
                .fill(CARD)
                .stroke(Stroke::new(1., if candidate { TEAL } else { LINE }))
                .corner_radius(8)
                .inner_margin(10)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        dot(ui, if connected(tracker) { TEAL } else { AMBER });
                        ui.label(RichText::new(&tracker.label).strong());
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            small(ui, &format!("{:.0} RX Hz", tracker.hz));
                        });
                    });
                    assignment_orientation_status(ui, tracker);
                    if candidate {
                        ui.label(
                            RichText::new("Movement match · awaiting your confirmation")
                                .small()
                                .color(TEAL),
                        );
                    }
                    let mut role = tracker.role.clone();
                    let role_text = role
                        .as_deref()
                        .and_then(|value| {
                            ROLES
                                .iter()
                                .find(|(id, _)| *id == value)
                                .map(|(_, label)| *label)
                        })
                        .unwrap_or("Choose body location");
                    egui::ComboBox::from_id_salt((&tracker.id, "role"))
                        .selected_text(role_text)
                        .width(ui.available_width())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut role, None, "Unassigned");
                            for (value, label) in ROLES {
                                ui.selectable_value(&mut role, Some((*value).to_owned()), *label);
                            }
                        });
                    if role != tracker.role {
                        self.handle.send(Command::Assign {
                            id: tracker.id.clone(),
                            role,
                        });
                    }
                });
        }
    }

    fn trackers(&mut self, ui: &mut egui::Ui, s: &Snapshot) {
        ui.label(RichText::new("Tracker inventory").size(20.).strong());
        small(
            ui,
            "Received orientations are measured; firmware-held orientations are compatibility estimates. Body position is estimated from the rig and anchors.",
        );
        small(
            ui,
            "Update counts describe received rotation packets. Resting firmware can suppress unchanged data; this is not the IMU sample rate.",
        );
        self.identification(ui, s);
        self.assignments(ui, s);
        for tracker in &s.trackers {
            egui::CollapsingHeader::new(format!("{} details", tracker.label))
                .id_salt((&tracker.id, "details"))
                .show(ui, |ui| {
                    small(
                        ui,
                        &format!(
                            "Firmware: {}",
                            if tracker.firmware.is_empty() {
                                "unreported"
                            } else {
                                &tracker.firmware
                            }
                        ),
                    );
                    small(
                        ui,
                        &format!(
                            "Rotation packets: {}   Orientation sample age: {:.2} s",
                            tracker.packets, tracker.age_seconds
                        ),
                    );
                    if let Some(age) = tracker.device_packet_age_seconds {
                        small(ui, &format!("Device packet age: {age:.2} s · {} datagrams · {} missing sequence numbers", tracker.device_packets, tracker.device_sequence_missing));
                    }
                    if let Some(age) = tracker.acceleration_age_seconds {
                        small(ui, &format!("Acceleration sample age: {age:.2} s · {} samples", tracker.acceleration_packets));
                    }
                    if tracker.estimated_held {
                        hint(ui, "Firmware-held orientation: the last real quaternion is reused under a firmware 0.7.2 continuity assumption. It is an estimate, not a fresh measurement; confidence is capped at 45%.", AMBER);
                    }
                    if let Some(level) = tracker.battery {
                        let pct = if level <= 1. { level * 100. } else { level };
                        small(ui, &format!("Battery: {:.0}%", pct.clamp(0., 100.)));
                    }
                    small(
                        ui,
                        &format!(
                            "Quaternion XYZW: {:.3}  {:.3}  {:.3}  {:.3}",
                            tracker.orientation[0],
                            tracker.orientation[1],
                            tracker.orientation[2],
                            tracker.orientation[3]
                        ),
                    );
                    if let Some(a) = tracker.acceleration {
                        small(
                            ui,
                            &format!("Acceleration: {:.2}  {:.2}  {:.2}", a[0], a[1], a[2]),
                        );
                    }
                    if let Some(age) = tracker.last_tap_seconds {
                        small(ui, &format!("Last detected tap: {age:.1} s ago"));
                    }
                });
        }
    }

    fn health(&self, ui: &mut egui::Ui, s: &Snapshot) {
        ui.label(RichText::new("System health").size(20.).strong());
        if ui.button("Save diagnostic snapshot").on_hover_text("Save the current status, firmware information and packet rates locally for validation.").clicked() {
            self.handle.send(Command::ExportDiagnostics);
        }
        hint(
            ui,
            &s.status,
            if s.trackers.is_empty() { AMBER } else { TEAL },
        );
        metric(ui, "Received packets", &s.packets_received.to_string());
        metric(ui, "Rejected packets", &s.malformed_packets.to_string());
        metric(
            ui,
            "Transport backlog resets",
            &s.transport_resets.to_string(),
        );
        metric(
            ui,
            "Missing sequence numbers",
            &s.sequence_missing.to_string(),
        );
        metric(
            ui,
            "Duplicate datagrams",
            &s.sequence_duplicates.to_string(),
        );
        metric(
            ui,
            "Out-of-order datagrams",
            &s.sequence_out_of_order.to_string(),
        );
        metric(ui, "Session", &format!("{:.0} min", s.uptime_seconds / 60.));
        metric(
            ui,
            "SteamVR",
            if s.steamvr_connected {
                "Connected"
            } else {
                "Not connected"
            },
        );
        ui.separator();
        ui.label(RichText::new("Estimator and timing").strong());
        for row in estimator_rows(s) {
            ui.horizontal(|ui| {
                small(ui, row.name);
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    ui.label(RichText::new(&row.value).strong());
                });
            })
            .response
            .on_hover_text(row.help);
        }
        small(
            ui,
            "Latency starts when this PC receives a datagram; Wi-Fi and firmware delay before that are not included. These are runtime counters, not accuracy measurements.",
        );
        ui.separator();
        ui.label(RichText::new("Connection and measurement age").strong());
        for tracker in &s.trackers {
            ui.label(RichText::new(&tracker.label).strong());
            let transport = tracker
                .device_packet_age_seconds
                .map(|age| format!("{age:.2} s"))
                .unwrap_or_else(|| "unknown".to_owned());
            small(
                ui,
                &format!(
                    "Device packet: {transport} · orientation sample: {:.2} s",
                    tracker.age_seconds
                ),
            );
            small(
                ui,
                &format!(
                    "{} device datagrams · {} rotations · {} acceleration samples",
                    tracker.device_packets, tracker.packets, tracker.acceleration_packets
                ),
            );
            if tracker.estimated_held {
                ui.label(
                    RichText::new(
                        "Firmware-held orientation · compatibility assumption · confidence ≤45%",
                    )
                    .small()
                    .color(AMBER),
                );
            }
        }
        if s.trackers.iter().any(|tracker| tracker.estimated_held) {
            hint(
                ui,
                "Firmware-held compatibility uses the last real quaternion from known firmware 0.7.2, healthy reported status and continuous device traffic. It expires after 1.5 seconds without transport, or on sequence gaps/reordering/errors until a new rotation. A heartbeat does not prove current orientation or IMU health. Confidence is capped at 45%; experimental drift correction skips held inputs.",
                AMBER,
            );
        }
        small(
            ui,
            "A connected device can hold an older orientation when firmware suppresses unchanged rotations. Connection and orientation freshness are separate.",
        );
        egui::CollapsingHeader::new("Packet type counters").show(ui, |ui| {
            for (kind, count) in &s.datagram_types {
                small(ui, &format!("Datagram type {kind}: {count}"));
            }
            for (kind, count) in &s.payload_types {
                small(ui, &format!("Payload {kind}: {count}"));
            }
        });
        ui.separator();
        ui.label(RichText::new("Output confidence").strong());
        for target in &s.targets {
            ui.horizontal(|ui| {
                dot(
                    ui,
                    if target.valid && !target.source.contains("FirmwareHeld") {
                        TEAL
                    } else {
                        AMBER
                    },
                );
                ui.label(&target.name);
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    small(ui, &format!("{:.0}%", target.confidence * 100.));
                });
            });
            small(
                ui,
                &format!(
                    "{} · {}",
                    if target.valid {
                        "Valid"
                    } else {
                        "Invalid / held"
                    },
                    target.source
                ),
            );
        }
        hint(
            ui,
            "An IMU measures orientation and acceleration. Foot contact and joint positions are hypotheses; a headset or controller provides an absolute anchor. Confidence is a heuristic, not an accuracy measurement.",
            MUTED,
        );
        ui.separator();
        ui.label(RichText::new("Recent messages").strong());
        if s.messages.is_empty() {
            small(ui, "No diagnostic messages.");
        }
        for message in s.messages.iter().rev().take(12) {
            small(ui, message);
        }
        if let Some(error) = &self.tray_error {
            hint(ui, error, AMBER);
        }
        egui::CollapsingHeader::new("Local profile").show(ui, |ui| {
            small(ui, &s.profile_path);
        });
    }

    fn controls(&mut self, ui: &mut egui::Ui, s: &Snapshot) {
        egui::Frame::new().fill(PANEL).corner_radius(12).inner_margin(14).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label(RichText::new("Tracking output").strong());
                    small(ui, &s.mode_status);
                });
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    let mut mode = s.output_mode;
                    egui::ComboBox::from_id_salt("output_mode").selected_text(mode.label()).width(170.).show_ui(ui, |ui| {
                        for option in OutputMode::ALL {
                            ui.selectable_value(&mut mode, option, option.label());
                        }
                    });
                    if mode != s.output_mode { self.handle.send(Command::SetOutputMode(mode)); }
                });
            });
            small(ui, "Auto: live in SteamVR whenever a worn headset is available and the calibration is valid, desktop preview otherwise. SteamVR / VRChat OSC: live only. Desktop only: ignores the headset and sends nothing. Nothing is sent during a calibration or from desktop preview poses.");
            ui.separator();
            ui.checkbox(&mut self.neutral_asserted,"I am upright and still, with head and torso facing together");
            ui.horizontal_wrapped(|ui| {
                if ui.add_enabled(s.calibrated && !s.desktop_session && !s.calibration_preview && !calibration_active(s) && self.neutral_asserted,egui::Button::new("Recover heading")).on_hover_text("Stand neutral, upright and still with head and torso facing in the same direction. Requests gated recovery; mounting calibration is kept.").clicked(){self.handle.send(Command::RequestRecovery);self.neutral_asserted=false;}
                if ui.add_enabled(s.calibrated, egui::Button::new("Undo recovery")).on_hover_text("Restore the previous heading correction after a recovery.").clicked() { self.handle.send(Command::UndoRecovery); }
                let mut gesture=s.gesture_enabled;
                if ui.checkbox(&mut gesture,"Hands-free tap gestures (chest / hip)").on_hover_text("Tap the chest or hip tracker firmly with your fingers while standing still. Double tap: heading recovery (stand neutral, still, head and torso facing together). Triple tap: start Calibrate by movement; triple tap again to cancel it. Results are spoken. Experimental: a tap is a request, never proof of a neutral pose; stillness and pose gates must still pass.").changed(){self.handle.send(Command::SetGesture(gesture));}
                if s.recording.is_some() {
                    if ui.button(RichText::new("Stop recording").color(AMBER)).clicked(){self.handle.send(Command::StopRecording);}
                } else if ui.button("Record session").on_hover_text("Save local measurements for reproducible validation.").clicked(){self.handle.send(Command::StartRecording);}
            });
            if s.gesture_enabled{small(ui,"Double tap: heading recovery (stand neutral and still, head and torso facing together). Triple tap: calibrate by movement, or cancel a running calibration.");}
            if s.gesture_enabled && !s.gesture_status.is_empty(){small(ui,&format!("Last gesture: {}",s.gesture_status));}
            if !s.recovery.is_empty(){small(ui,&format!("Recovery: {}",s.recovery));}
            if s.desktop_session { small(ui,"Use Recalibrate neutral pose to recenter the desktop preview."); }
            self.estimator_controls(ui, s);
            if let Some(path)=&s.recording { small(ui,&format!("Recording locally: {path}")); }
        });
    }

    /// Estimator switches. Every change is one `SetEstimator` with the full
    /// settings; the runtime validates, persists and echoes them back.
    fn estimator_controls(&mut self, ui: &mut egui::Ui, s: &Snapshot) {
        let current = s.estimator_settings;
        if !self.prediction_lead_dirty {
            self.prediction_lead_ms = current.prediction_lead_ms;
        }
        egui::CollapsingHeader::new("Estimator · experimental")
            .id_salt("estimator_settings")
            .show(ui, |ui| {
                small(
                    ui,
                    "Each switch has been checked on synthetic motion only, not against worn hardware. Turn one off to compare. Changes are unavailable during a capture countdown.",
                );
                let mut next = current;
                ui.add_enabled_ui(!calibration_active(s), |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.checkbox(&mut next.stance_heading, "Stance heading filter")
                            .on_hover_text("Estimates each sensor's heading drift by comparing how the headset actually moved with how the leg chain says it moved while a foot is standing still. Needs a live headset and some movement; inactive in desktop preview. A foot that slowly slides, or a mounting error, can be mistaken for drift. Experimental.");
                        ui.checkbox(&mut next.leg_ik, "Contact-aware legs")
                            .on_hover_text("Locks a planted foot horizontally and shifts the hips to stay consistent with it. The foot's height follows the body model, except that an upright leg near the floor is held at floor height. A planted foot without a foot tracker is shown flat while its shin is near upright. Seated and lying legs keep the knee angle their sensors measure; a squat with the torso bowed forward is treated as standing. Experimental: foot contact is inferred from motion, not measured, so a wrong guess can pin a moving foot briefly; sitting and lying are not validated on a worn body. Sensor rotations are never changed; positions stay inferred.");
                        ui.checkbox(&mut next.prediction, "Motion prediction")
                            .on_hover_text("Extrapolates each sensor along its measured rotation rate to the moment of solving, so Wi-Fi delay shows less. Only recent samples are extrapolated: silence from a tracker means 'still', never 'keep turning'. Sudden stops can overshoot briefly.");
                        ui.checkbox(&mut next.hmd_extrapolation, "Headset extrapolation")
                            .on_hover_text("Advances the headset and controller positions along their reported velocity by their age (at most 30 ms). Orientation is not extrapolated. Assumes SteamVR's velocity is current; a tracking glitch can push the body briefly.");
                        ui.checkbox(&mut next.auto_rebase, "Recenter-proof calibration")
                            .on_hover_text("When the headset jumps (a SteamVR recenter or tracking jump), treats it as a turn of the play space and carries calibration over instead of leaving the body rotated. A genuine fast movement misread as a jump could rotate the body; the carry-over count in Health shows when it acted.");
                        ui.checkbox(&mut next.angular_velocity_output, "Send angular velocity to SteamVR")
                            .on_hover_text("Lets SteamVR extrapolate tracker rotation between updates. Off by default: the reference frame SteamVR expects for this field is unverified, so a wrong frame would make trackers spin or wobble during motion.");
                    });
                    ui.horizontal(|ui| {
                        let response = ui
                            .add_enabled(
                                next.prediction,
                                egui::Slider::new(&mut self.prediction_lead_ms, 0.0..=30.0)
                                    .step_by(1.)
                                    .fixed_decimals(0)
                                    .suffix(" ms")
                                    .text("Prediction lead"),
                            )
                            .on_hover_text("Predicts this much further ahead than the solve time, to hide part of the remaining output delay. 0 ms is the safest setting; larger leads overshoot more when you stop suddenly.")
                            .on_disabled_hover_text("Turn on motion prediction to use a lead.");
                        self.prediction_lead_dirty |= response.changed();
                        // Each change re-validates and saves settings: commit
                        // once on release or focus loss, as the height does.
                        if self.prediction_lead_dirty
                            && (response.drag_stopped()
                                || response.lost_focus()
                                || (!response.dragged() && !response.has_focus()))
                        {
                            self.prediction_lead_dirty = false;
                            if self.prediction_lead_ms.is_finite() {
                                next.prediction_lead_ms = self.prediction_lead_ms.clamp(0., 30.);
                            }
                        }
                    });
                });
                if next != current {
                    self.handle.send(Command::SetEstimator(next));
                }
                ui.horizontal_wrapped(|ui| {
                    let mut correction = s.experimental_correction;
                    if ui
                        .checkbox(&mut correction, "Experimental drift correction")
                        .on_hover_text("Older, conservative heading adjustment of the thighs from knee-hinge motion. Gated, experimental and needs live validation. When the stance heading filter is enabled, it supersedes this correction.")
                        .changed()
                    {
                        self.handle.send(Command::SetCorrection(correction));
                    }
                    if !s.correction.is_empty() {
                        small(ui, &s.correction);
                    }
                });
                small(ui, &estimator_summary(s));
            });
    }

    fn main_view(&mut self, ui: &mut egui::Ui, s: &Snapshot, available_height: f32) {
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label(RichText::new("Movement studio").size(23.).strong());
                small(
                    ui,
                    if !s.calibrated {
                        "Received / firmware-held orientations · body setup required"
                    } else if s.calibration_preview && s.desktop_motion_enabled {
                        "Desktop body preview · experimental estimated movement"
                    } else if s.calibration_preview {
                        "Desktop body preview · fixed center and saved height"
                    } else if s.trackers.iter().any(|tracker| tracker.estimated_held) {
                        "Estimated body pose · firmware-held compatibility input"
                    } else {
                        "Estimated body pose · measured sensor orientations"
                    },
                );
            });
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                ui.checkbox(&mut self.show_scene, "Show 3D view");
            });
        });
        self.calibration_feedback(ui, s);
        ui.add_space(4.);
        egui::Frame::new()
            .fill(PANEL)
            .corner_radius(14)
            .stroke(Stroke::new(1., LINE))
            .inner_margin(12)
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    let facing = preview_facing(s).map_or(-Vec3::Z, |(direction, _)| direction);
                    let front_yaw = facing.x.atan2(facing.z);
                    for (title, yaw, pitch) in [
                        ("Perspective", std::f32::consts::PI + 0.65, 0.20),
                        ("Front", front_yaw, 0.05),
                        ("Behind", front_yaw + std::f32::consts::PI, 0.05),
                        ("Side", front_yaw + std::f32::consts::FRAC_PI_2, 0.05),
                    ] {
                        if ui.small_button(title).on_hover_text("Changes only the preview camera, not calibration or tracking.").clicked() {
                            self.camera.yaw = yaw;
                            self.camera.pitch = pitch;
                        }
                    }
                    if ui.small_button("Reset view").clicked() {
                        self.camera = Camera::default();
                    }
                    ui.checkbox(&mut self.show_axes, "Sensor axes");
                    badge(
                        ui,
                        if s.output_sink == OutputSink::Off { "OUTPUT OFF" } else { "LIVE OUTPUT" },
                        if s.output_sink == OutputSink::Off { MUTED } else { AMBER },
                    );
                });
                let size = Vec2::new(ui.available_width(), (available_height - 330.).max(180.));
                let (rect, response) = ui.allocate_exact_size(size, Sense::click_and_drag());
                response.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Image, ui.is_enabled(), "Interactive body preview. Drag to orbit, right-drag to pan, and scroll to zoom."));
                if self.show_scene {
                    self.camera.interact(ui, &response);
                    draw_scene(ui, &rect, &self.camera, s, self.show_axes, &self.manual_role);
                } else {
                    ui.painter().text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        "Preview paused · tracking continues",
                        FontId::proportional(18.),
                        MUTED,
                    );
                }
                ui.horizontal_wrapped(|ui| {
                    small(ui, "Camera only · drag to orbit");
                    ui.separator();
                    small(ui, "Right-drag to pan");
                    ui.separator();
                    small(ui, "Scroll to zoom");
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        small(ui, "Standing space · meters · Y up");
                    });
                });
            });
        ui.add_space(4.);
        self.controls(ui, s);
    }
}

impl eframe::App for ChadApp {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let state = self.handle.snapshot();
        if state.calibration_request.phase == CalibrationRequestPhase::Captured {
            match state.calibration_request_kind {
                CalibrationRequestKind::StandingHeight
                    if self.last_height_capture_id != state.calibration_request.request_id =>
                {
                    self.height = state.height_m;
                    self.last_height_capture_id = state.calibration_request.request_id;
                }
                CalibrationRequestKind::NeutralPose | CalibrationRequestKind::MotionPose
                    if self.last_neutral_capture_id != state.calibration_request.request_id =>
                {
                    self.camera = Camera::default();
                    if let Some(hip) = state.joints.iter().find(|joint| joint.name == "Hip") {
                        let position = Vec3::from_array(hip.position);
                        if position.is_finite() {
                            self.camera.target = position;
                        }
                    }
                    self.last_neutral_capture_id = state.calibration_request.request_id;
                }
                _ => {}
            }
        }
        if let Some(session) = &mut self.identification_session
            && session.outcome().is_none()
        {
            let snapshot = self.handle.snapshot();
            session.observe(
                self.identification_clock.elapsed().as_secs_f64(),
                &identification_observations(&snapshot),
            );
        }
        if crate::single_instance::take_show_request() {
            ctx.send_viewport_cmd(egui::ViewportCommand::Visible(true));
            ctx.send_viewport_cmd(egui::ViewportCommand::Minimized(false));
            ctx.send_viewport_cmd(egui::ViewportCommand::Focus);
        }
        while let Ok(action) = self.tray_events.try_recv() {
            match action {
                TrayAction::Show => {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Visible(true));
                    ctx.send_viewport_cmd(egui::ViewportCommand::Focus);
                }
                TrayAction::Recover => {
                    // Open the recovery controls. A tray click cannot assert the
                    // neutral pose on behalf of the user.
                    self.neutral_asserted = false;
                    ctx.send_viewport_cmd(egui::ViewportCommand::Visible(true));
                    ctx.send_viewport_cmd(egui::ViewportCommand::Focus);
                }
                TrayAction::Exit => self.quit(ctx),
            }
        }
        if ctx.input(|i| i.viewport().close_requested()) && !self.quitting {
            if self.tray.is_some() {
                ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
                ctx.send_viewport_cmd(egui::ViewportCommand::Visible(false));
            } else {
                self.handle.send(Command::Shutdown);
            }
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let snapshot = self.handle.snapshot();
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(BG).inner_margin(20))
            .show(ui, |ui| {
                if !self.quitting {
                    worker_banner(ui, &self.handle.worker_status());
                }
                self.header(ui, &snapshot);
                ui.add_space(10.);
                ui.separator();
                ui.add_space(6.);
                let body_height = ui.available_height();
                ui.horizontal_top(|ui| {
                    egui::Frame::new()
                        .fill(PANEL)
                        .corner_radius(14)
                        .inner_margin(16)
                        .show(ui, |ui| {
                            // Frames inherit the surrounding layout; this one
                            // sits in a horizontal row and needs its own column.
                            ui.with_layout(Layout::top_down(Align::Min), |ui| {
                                ui.set_width(322.);
                                ui.set_min_height(body_height - 32.);
                                self.sidebar(ui, &snapshot);
                            });
                        });
                    ui.add_space(4.);
                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width());
                        let available_height = ui.available_height();
                        // Small restored windows retain access to recovery and
                        // recording controls instead of clipping the footer.
                        egui::ScrollArea::vertical()
                            .id_salt("studio_scroll")
                            .auto_shrink([false, false])
                            .show(ui, |ui| self.main_view(ui, &snapshot, available_height));
                    });
                });
            });
        ctx.request_repaint_after(Duration::from_millis(33));
    }
}

/// The snapshot below a stopped or stalled worker is frozen; say so loudly.
fn worker_banner(ui: &mut egui::Ui, status: &WorkerStatus) {
    let text = match status {
        WorkerStatus::Running => return,
        WorkerStatus::Stalled(seconds) => format!(
            "TRACKING WORKER NOT RESPONDING for {seconds:.0} s. Everything shown below is frozen and no tracking is being sent."
        ),
        WorkerStatus::Stopped(Some(reason)) => format!(
            "TRACKING WORKER CRASHED: {reason}. Nothing below is live and no tracking is being sent. Restart ChadFBT."
        ),
        WorkerStatus::Stopped(None) => {
            "TRACKING WORKER STOPPED. Nothing below is live and no tracking is being sent. Restart ChadFBT.".to_owned()
        }
    };
    egui::Frame::new()
        .fill(Color32::from_rgb(120, 24, 32))
        .corner_radius(8)
        .inner_margin(12)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(text).strong().color(Color32::WHITE));
        });
    ui.add_space(8.);
}

impl Drop for ChadApp {
    fn drop(&mut self) {
        self.handle.send(Command::Shutdown);
    }
}

fn create_tray(
    ctx: &egui::Context,
    tx: mpsc::Sender<TrayAction>,
) -> Result<TrayIcon, Box<dyn std::error::Error>> {
    let menu = Menu::new();
    let show = MenuItem::new("Show ChadFBT", true, None);
    let recovery = MenuItem::new("Open heading recovery...", true, None);
    let exit = MenuItem::new("Exit ChadFBT", true, None);
    menu.append_items(&[&show, &recovery, &PredefinedMenuItem::separator(), &exit])?;
    let show_id = show.id().clone();
    let recovery_id = recovery.id().clone();
    let exit_id = exit.id().clone();
    let menu_ctx = ctx.clone();
    let menu_tx = tx.clone();
    MenuEvent::set_event_handler(Some(move |event: MenuEvent| {
        let action = if event.id == show_id {
            Some(TrayAction::Show)
        } else if event.id == recovery_id {
            Some(TrayAction::Recover)
        } else if event.id == exit_id {
            Some(TrayAction::Exit)
        } else {
            None
        };
        if let Some(action) = action {
            let _ = menu_tx.send(action);
            menu_ctx.request_repaint();
        }
    }));
    let tray_ctx = ctx.clone();
    TrayIconEvent::set_event_handler(Some(move |event: TrayIconEvent| {
        if matches!(
            event,
            TrayIconEvent::DoubleClick { .. }
                | TrayIconEvent::Click {
                    button: tray_icon::MouseButton::Left,
                    button_state: tray_icon::MouseButtonState::Up,
                    ..
                }
        ) {
            let _ = tx.send(TrayAction::Show);
            tray_ctx.request_repaint();
        }
    }));
    let mut rgba = vec![0u8; 32 * 32 * 4];
    for y in 0..32 {
        for x in 0..32 {
            let index = (y * 32 + x) * 4;
            let dx = x as f32 - 15.5;
            let dy = y as f32 - 15.5;
            let inside = dx * dx + dy * dy < 220.;
            let glyph = ((8..=11).contains(&x) && (8..=23).contains(&y))
                || ((8..=11).contains(&y) && (8..=23).contains(&x))
                || ((20..=23).contains(&y) && (8..=23).contains(&x));
            let color = if glyph { TEAL } else { BG };
            rgba[index..index + 4].copy_from_slice(&[
                color.r(),
                color.g(),
                color.b(),
                if inside { 255 } else { 0 },
            ]);
        }
    }
    let icon = tray_icon::Icon::from_rgba(rgba, 32, 32)?;
    Ok(TrayIconBuilder::new()
        .with_menu(Box::new(menu))
        .with_tooltip("ChadFBT · click to open")
        .with_icon(icon)
        .build()?)
}

fn section(ui: &mut egui::Ui, index: &str, title: &str, body: impl FnOnce(&mut egui::Ui)) {
    ui.add_space(5.);
    ui.horizontal(|ui| {
        ui.label(RichText::new(index).color(TEAL).strong());
        ui.label(RichText::new(title).size(16.).strong());
    });
    body(ui);
    ui.add_space(7.);
    ui.separator();
}
const MOVEMENT_INSTRUCTIONS: &str = "After the tone: bow forward once as if greeting someone, then lift each knee twice (or do two slow squats). Keep facing the same direction, knees pointing forward, no twisting. You do not need to hold any pose.";

/// One-line honest limits, always visible under the movement button.
fn limits_note(preview: bool) -> &'static str {
    if preview {
        "Limits (hover): a sideways lean cannot be detected in desktop preview, and knees pointing outward bias the legs."
    } else {
        "Limits (hover): knees pointing outward bias the legs; the stance heading filter refines this while you move afterwards."
    }
}

fn limits_detail(preview: bool) -> &'static str {
    if preview {
        "Movement calibration assumes each tracker moves forward and back in one plane. In desktop preview: a sideways lean of the torso or a leg looks exactly like a different tracker direction and cannot be detected. Knees or toes pointing outward during squats bias the leg directions by several degrees, and nothing corrects this afterwards in desktop preview. Turning during the movement is detected; swapped trackers are only suspected. Validated on synthetic data only, not on a worn body."
    } else {
        "Movement calibration assumes each tracker moves forward and back in one plane. A sideways leg movement looks like a different tracker direction; only a sideways torso lean is exposed, by the headset's movement. Knees or toes pointing outward during squats bias the leg directions by several degrees; with the headset, the stance heading filter (if enabled) refines this while you move afterwards. Turning during the movement is detected; swapped trackers are only suspected. Validated on synthetic data only, not on a worn body."
    }
}

/// The segment's current blocker in plain words.
fn motion_segment_hint(segment: &MotionSegmentStatus) -> String {
    if segment.ready {
        return "ready".into();
    }
    let Some(blocker) = segment.blockers.first() else {
        return "keep moving".into();
    };
    match blocker.gate {
        MotionGate::NotStarted => "waiting to start".into(),
        MotionGate::NoData => "no data from this tracker".into(),
        MotionGate::Stale => "tracker data stopped".into(),
        MotionGate::Excitation => match segment.role {
            Role::Chest => "tilt further: bow deeper".into(),
            Role::Hip => "tilt further: push the hips back".into(),
            Role::LeftThigh | Role::RightThigh => "tilt further: lift the knee higher".into(),
            _ => "tilt further: bend the knee forward".into(),
        },
        MotionGate::Planarity => "keep it in one plane".into(),
        MotionGate::OneSidedness => "bend one way only, no swinging".into(),
        MotionGate::Repeatability if segment.repeatability_degrees.is_none() => {
            "wait: checking consistency".into()
        }
        MotionGate::Repeatability => "repeat the same movement".into(),
        MotionGate::ModelResidual => "forward and back only, not sideways".into(),
        MotionGate::Stability => "keep going: still settling".into(),
        MotionGate::KneeHyperextension => "knee bends backward: check assignment".into(),
        MotionGate::KneeAxis => "thigh and calf disagree: face one way".into(),
        MotionGate::KneeUnchecked => "wait: checking the knee".into(),
        MotionGate::BodyTurn => "you turned: face one way".into(),
        MotionGate::Reference => "no forward reference".into(),
    }
}

fn motion_role_label(role: Role) -> &'static str {
    match role {
        Role::Chest => "Chest",
        Role::Hip => "Hip",
        Role::LeftThigh => "L thigh",
        Role::RightThigh => "R thigh",
        Role::LeftCalf => "L calf",
        Role::RightCalf => "R calf",
        _ => "Other",
    }
}

fn reference_name(reference: MotionReference) -> &'static str {
    match reference {
        MotionReference::HeadsetYaw => "head direction at neutral",
        MotionReference::Fused => "both combined",
        MotionReference::NeckDisplacement => "neck movement",
    }
}

/// Headset forward diagnostics (live mode only).
fn headset_forward_text(status: &MotionCalibrationStatus) -> Option<String> {
    let headset = status.headset.as_ref()?;
    let used = reference_name(status.selected_reference);
    Some(match headset.difference_from_yaw_radians {
        Some(difference) if headset.available => format!(
            "Headset forward: neck movement points {:+.1}° from the head direction at neutral (±{:.0}°){}; using {used}.",
            difference.to_degrees(),
            headset
                .displacement_sigma_radians
                .unwrap_or(0.)
                .to_degrees(),
            if headset.disagreement_flagged {
                ", too far apart to combine"
            } else {
                ""
            },
        ),
        _ => format!(
            "Headset forward: {}; using {used}.",
            headset
                .reason
                .as_deref()
                .unwrap_or("neck movement not measured yet")
        ),
    })
}

/// Per-segment progress rows, warnings and (live) headset diagnostics.
fn motion_progress(ui: &mut egui::Ui, s: &Snapshot) {
    let Some(status) = &s.motion_calibration else {
        return;
    };
    egui::Grid::new("motion_calibration_rows")
        .num_columns(3)
        .spacing(egui::vec2(8., 4.))
        .show(ui, |ui| {
            for segment in &status.segments {
                ui.label(RichText::new(motion_role_label(segment.role)).small());
                let progress = if segment.progress.is_finite() {
                    segment.progress.clamp(0., 1.)
                } else {
                    0.
                };
                ui.add(
                    egui::ProgressBar::new(progress)
                        .desired_width(110.)
                        .desired_height(10.)
                        .fill(if segment.ready { TEAL } else { AMBER }),
                );
                let detail = segment
                    .blockers
                    .first()
                    .map_or("Ready.", |blocker| blocker.message.as_str());
                ui.label(
                    RichText::new(motion_segment_hint(segment))
                        .small()
                        .color(if segment.ready { TEAL } else { MUTED }),
                )
                .on_hover_text(detail);
                ui.end_row();
            }
        });
    for blocker in &status.blockers {
        if blocker.gate != MotionGate::NotStarted {
            hint(ui, &blocker.message, AMBER);
        }
    }
    let mut shown = Vec::new();
    for warning in &status.warnings {
        // The headset disagreement is a live-mode diagnostic; body turns are
        // already shown as a calibration-wide blocker.
        let skip = (s.desktop_session && warning.kind == MotionWarningKind::HeadsetDisagreement)
            || (warning.kind == MotionWarningKind::BodyTurn
                && status
                    .blockers
                    .iter()
                    .any(|b| b.gate == MotionGate::BodyTurn));
        if skip || shown.contains(&warning.message) {
            continue;
        }
        let title = match warning.kind {
            MotionWarningKind::BodyTurn => "Turned during the procedure",
            MotionWarningKind::SuspectedThighCalfSwap => "Thigh and calf trackers may be swapped",
            MotionWarningKind::SuspectedLeftRightSwap => "Left and right trackers may be swapped",
            MotionWarningKind::KneeHyperextension => "Knee would bend backward",
            MotionWarningKind::KneeAxisMismatch => "Thigh and calf bend in different directions",
            MotionWarningKind::SignFromKneeRule => "Direction chosen from the knee's bend",
            MotionWarningKind::HeadsetDisagreement => {
                "Head and torso may not have faced the same way"
            }
        };
        hint(ui, &format!("{title}. {}", warning.message), AMBER);
        shown.push(warning.message.clone());
    }
    if !s.desktop_session
        && let Some(text) = headset_forward_text(status)
    {
        small(ui, &text);
    }
}

// A side-view guide only: capture still relies on the user to adopt the pose.
fn draw_forward_bend_guide(ui: &mut egui::Ui) {
    let (rect, response) =
        ui.allocate_exact_size(egui::vec2(ui.available_width(), 130.), Sense::hover());
    response.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Image, ui.is_enabled(),
        "Side view of a shallow forward skiing pose: feet flat, knees forward, hips back, torso leaning forward."));
    let p = ui.painter();
    p.rect_filled(rect, 8, CARD);
    let point = |x: f32, y: f32| rect.left_top() + egui::vec2(x * rect.width(), y * rect.height());
    let head = point(0.52, 0.17);
    let shoulder = point(0.49, 0.30);
    let hip = point(0.39, 0.60);
    let knee = point(0.52, 0.72);
    let ankle = point(0.47, 0.90);
    let toe = point(0.59, 0.90);
    p.line_segment(
        [point(0.23, 0.91), point(0.75, 0.91)],
        Stroke::new(1., MUTED),
    );
    p.circle_stroke(head, 7., Stroke::new(3., TEAL));
    for (a, b) in [(shoulder, hip), (hip, knee), (knee, ankle), (ankle, toe)] {
        p.line_segment([a, b], Stroke::new(4., TEAL));
    }
    p.line_segment([shoulder, point(0.47, 0.55)], Stroke::new(3., MUTED));
    p.arrow(
        point(0.66, 0.27),
        egui::vec2(rect.width() * 0.15, 0.),
        Stroke::new(2., AMBER),
    );
    p.text(
        point(0.65, 0.11),
        egui::Align2::LEFT_TOP,
        "FORWARD",
        FontId::proportional(10.),
        AMBER,
    );
    p.text(
        point(0.05, 0.12),
        egui::Align2::LEFT_TOP,
        "SIDE VIEW",
        FontId::proportional(10.),
        MUTED,
    );
}

fn draw_raw_sensor_axes(ui: &mut egui::Ui, tracker: &TrackerView) {
    let (rect, response) =
        ui.allocate_exact_size(egui::vec2(ui.available_width(), 88.), Sense::hover());
    response.widget_info(|| {
        egui::WidgetInfo::labeled(
            egui::Role::Image,
            ui.is_enabled(),
            "Selected tracker's raw sensor axes: X red, Y green, Z blue.",
        )
    });
    let p = ui.painter().with_clip_rect(rect);
    p.rect_filled(rect, 8, CARD);
    p.text(
        rect.left_top() + egui::vec2(10., 8.),
        egui::Align2::LEFT_TOP,
        "RAW SENSOR AXES",
        FontId::proportional(10.),
        MUTED,
    );
    let camera = Camera {
        yaw: 2.35,
        pitch: 0.45,
        distance: 2.4,
        target: Vec3::ZERO,
    };
    draw_axes(&p, &camera, &rect, Vec3::ZERO, tracker.orientation, 0.65);
    p.text(
        rect.right_bottom() - egui::vec2(10., 8.),
        egui::Align2::RIGHT_BOTTOM,
        "X red · Y green · Z blue",
        FontId::proportional(10.),
        MUTED,
    );
}

// Each handle edits a real physical-trim component. The diagram displays that
// local trim, while the separate raw-axes display above shows sensor evidence.
fn draw_manual_gizmo(ui: &mut egui::Ui, degrees: &mut [f32]) -> bool {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(ui.available_width(), 126.), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 8, CARD);
    p.text(
        rect.left_top() + egui::vec2(10., 8.),
        egui::Align2::LEFT_TOP,
        "LOCAL TRIM · DRAG AXIS HANDLE",
        FontId::proportional(10.),
        MUTED,
    );
    let q = Quat::from_rotation_x(degrees[0].to_radians())
        * Quat::from_rotation_y(degrees[1].to_radians())
        * Quat::from_rotation_z(degrees[2].to_radians());
    let camera = Camera {
        yaw: 2.35,
        pitch: 0.45,
        distance: 2.4,
        target: Vec3::ZERO,
    };
    let (_, _, right, up) = camera.basis();
    let center = rect.center() + egui::vec2(0., 10.);
    let mut committed = false;
    for (index, (axis, label, color)) in [
        (Vec3::X, "X", Color32::from_rgb(240, 105, 120)),
        (Vec3::Y, "Y", TEAL),
        (Vec3::Z, "Z", Color32::from_rgb(101, 162, 255)),
    ]
    .into_iter()
    .enumerate()
    {
        let rotated = q * axis;
        let end = center + egui::vec2(rotated.dot(right), -rotated.dot(up)) * 43.;
        p.line_segment([center, end], Stroke::new(3., color));
        p.circle_filled(end, 10., color);
        p.text(
            end,
            egui::Align2::CENTER_CENTER,
            label,
            FontId::proportional(11.),
            BG,
        );
        let handle = ui.interact(
            egui::Rect::from_center_size(end, egui::vec2(24., 24.)),
            ui.make_persistent_id(("mount_handle", index)),
            Sense::drag(),
        );
        handle.widget_info(|| {
            egui::WidgetInfo::labeled(
                egui::Role::Button,
                ui.is_enabled(),
                format!("Drag {label} physical mounting rotation"),
            )
        });
        if handle.dragged() {
            let delta = ui.input(|i| i.pointer.delta());
            degrees[index] = (degrees[index] + (delta.x - delta.y) * 0.5).clamp(-180., 180.);
        }
        committed |= handle.drag_stopped();
    }
    committed
}

fn small(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).small().color(MUTED));
}
fn hint(ui: &mut egui::Ui, text: &str, color: Color32) {
    egui::Frame::new()
        .fill(color.gamma_multiply(0.07))
        .corner_radius(8)
        .inner_margin(10)
        .show(ui, |ui| {
            ui.label(RichText::new(text).small().color(color));
        });
}
fn primary(ui: &mut egui::Ui, text: &str, enabled: bool) -> egui::Response {
    ui.add_enabled(
        enabled,
        egui::Button::new(RichText::new(text).color(BG).strong())
            .fill(TEAL)
            .min_size(egui::vec2(ui.available_width(), 36.)),
    )
}
fn badge(ui: &mut egui::Ui, text: &str, color: Color32) {
    egui::Frame::new()
        .fill(color.gamma_multiply(0.09))
        .stroke(Stroke::new(1., color.gamma_multiply(0.3)))
        .corner_radius(6)
        .inner_margin(egui::vec2(8., 4.))
        .show(ui, |ui| {
            ui.label(RichText::new(text).size(10.).strong().color(color));
        });
}
fn dot(ui: &mut egui::Ui, color: Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(8., 8.), Sense::hover());
    ui.painter().circle_filled(rect.center(), 3., color);
}
fn metric(ui: &mut egui::Ui, name: &str, value: &str) {
    ui.horizontal(|ui| {
        small(ui, name);
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            ui.label(RichText::new(value).strong());
        });
    });
}

/// Viewport ankle tints for the inferred planted-foot hypothesis and a leg IK lock.
const PLANTED_TINT: Color32 = Color32::from_rgb(120, 190, 90);
const LOCKED_TINT: Color32 = Color32::from_rgb(140, 110, 230);

/// One line of the Health page's estimator block.
struct EstimatorRow {
    name: &'static str,
    value: String,
    help: &'static str,
}

/// Estimator and timing diagnostics, in display order. To show a new
/// `EstimatorStatus` field, push one more row here; Health renders the list.
fn estimator_rows(s: &Snapshot) -> Vec<EstimatorRow> {
    let e = &s.estimator;
    let settings = &s.estimator_settings;
    let latency = match (s.packet_to_solve_p50_ms, s.packet_to_solve_p95_ms) {
        (Some(p50), Some(p95)) => format!("{} / {} ms", finite(p50, 1), finite(p95, 1)),
        _ => "no samples".to_owned(),
    };
    let rebases = if e.standing_space_rebases == 0 {
        "0".to_owned()
    } else {
        format!(
            "{} · last {}°",
            e.standing_space_rebases,
            finite(e.last_rebase_degrees, 1)
        )
    };
    let mut rows = vec![
        EstimatorRow {
            name: "Solve rate",
            value: format!("{} Hz", finite(s.solve_rate_hz, 0)),
            help: "Solves (and SteamVR publishes) completed during the last second.",
        },
        EstimatorRow {
            name: "Solve time",
            value: format!("{} us", finite(s.solve_micros as f32, 0)),
            help: "Time spent computing the latest body pose.",
        },
        EstimatorRow {
            name: "Packet to solve p50 / p95",
            value: latency,
            help: "From datagram receipt on this PC to the solve that used it, over the last second.",
        },
        EstimatorRow {
            name: "Receive drops",
            value: s.receive_dropped.to_string(),
            help: "Datagrams dropped because the worker fell behind. They also appear as missing sequence numbers.",
        },
        EstimatorRow {
            name: "Motion prediction",
            value: if !settings.prediction {
                "Off".to_owned()
            } else if e.prediction_active {
                "Active".to_owned()
            } else {
                "Idle (no fresh rotation rate)".to_owned()
            },
            help: "Active when at least one sensor orientation was extrapolated from a fresh measured rotation rate in the latest frame.",
        },
        EstimatorRow {
            name: "Headset extrapolation",
            value: if settings.hmd_extrapolation {
                format!("{} ms", finite(e.hmd_extrapolated_secs * 1000., 1))
            } else {
                "Off".to_owned()
            },
            help: "How far the headset position was advanced along its reported velocity in the latest frame.",
        },
        EstimatorRow {
            name: "Recenter carry-overs",
            value: rebases,
            help: "Explicit and automatic play-space rebases since start, with the turn of the latest one about the vertical axis.",
        },
        EstimatorRow {
            name: "Tracking discontinuities",
            value: e.tracking_discontinuities.to_string(),
            help: "Headset jumps found by the automatic detector, including ones that could not be carried over (no usable horizontal facing).",
        },
    ];
    push_stance_rows(&mut rows, s);
    push_contact_rows(&mut rows, s);
    push_leg_ik_rows(&mut rows, s);
    rows
}

/// Radians to a signed degree string; "n/a" when nonfinite.
fn signed_degrees(radians: f32) -> String {
    if radians.is_finite() {
        format!("{:+.1}°", radians.to_degrees())
    } else {
        "n/a".to_owned()
    }
}

fn unsigned_degrees(radians: f32) -> String {
    if radians.is_finite() {
        format!("{:.1}°", radians.abs().to_degrees())
    } else {
        "n/a".to_owned()
    }
}

fn phase_name(phase: tracking_core::contact::ContactPhase) -> &'static str {
    use tracking_core::contact::ContactPhase;
    match phase {
        ContactPhase::Swing => "swing",
        ContactPhase::Settling => "settling",
        ContactPhase::Planted => "planted",
    }
}

fn push_stance_rows(rows: &mut Vec<EstimatorRow>, s: &Snapshot) {
    use tracking_core::stance_heading::StanceHeadingStatus as St;
    let e = &s.estimator;
    let status = if !s.estimator_settings.stance_heading {
        "Off".to_owned()
    } else if !e.stance_active {
        "Inactive (needs live headset and a usable leg chain)".to_owned()
    } else {
        match e.stance.status {
            St::Idle => "Active · idle (no planted foot)",
            St::Observing => "Active · observing planted foot",
            St::Updated => "Active · updated",
            St::Rejected => "Active · measurements rejected",
            St::Discontinuity => "Active · headset discontinuity",
            St::InvalidInput => "Active · invalid input ignored",
        }
        .to_owned()
    };
    rows.push(EstimatorRow {
        name: "Stance heading filter",
        value: status,
        help: "Whether the heading-drift filter ran this frame and what it did. 'Idle' just means no foot is standing still, so nothing can be learned. This describes the filter's own state, not tracking accuracy.",
    });
    const NAMES: [&str; 6] = [
        "Heading correction · chest",
        "Heading correction · hip",
        "Heading correction · L thigh",
        "Heading correction · R thigh",
        "Heading correction · L calf",
        "Heading correction · R calf",
    ];
    const HELPS: [&str; 6] = [
        "Estimated yaw drift removed from the chest sensor, with the filter's 1-sigma uncertainty. The uncertainty is the model's own consistency estimate, not a measurement of tracking accuracy.",
        "Estimated yaw drift removed from the hip sensor, with the filter's 1-sigma uncertainty. The uncertainty is the model's own consistency estimate, not a measurement of tracking accuracy.",
        "Estimated yaw drift removed from the left thigh sensor, with the filter's 1-sigma uncertainty. The uncertainty is the model's own consistency estimate, not a measurement of tracking accuracy.",
        "Estimated yaw drift removed from the right thigh sensor, with the filter's 1-sigma uncertainty. The uncertainty is the model's own consistency estimate, not a measurement of tracking accuracy.",
        "Estimated yaw drift removed from the left calf sensor, with the filter's 1-sigma uncertainty. The uncertainty is the model's own consistency estimate, not a measurement of tracking accuracy.",
        "Estimated yaw drift removed from the right calf sensor, with the filter's 1-sigma uncertainty. The uncertainty is the model's own consistency estimate, not a measurement of tracking accuracy.",
    ];
    for i in 0..6 {
        rows.push(EstimatorRow {
            name: NAMES[i],
            value: format!(
                "{} ± {}",
                signed_degrees(e.stance.corrections[i]),
                unsigned_degrees(e.stance.sigma[i])
            ),
            help: HELPS[i],
        });
    }
}

fn push_contact_rows(rows: &mut Vec<EstimatorRow>, s: &Snapshot) {
    let e = &s.estimator;
    const PHASE: [&str; 2] = ["L foot contact", "R foot contact"];
    const SPEED: [&str; 2] = ["L ankle speed / allowance", "R ankle speed / allowance"];
    const MEAS: [&str; 2] = ["L foot heading measurements", "R foot heading measurements"];
    const INNOV: [&str; 2] = ["L foot last innovation", "R foot last innovation"];
    for f in 0..2 {
        let contact = &e.contacts[f];
        let diag = &e.stance.feet[f];
        rows.push(EstimatorRow {
            name: PHASE[f],
            value: phase_name(contact.phase).to_owned(),
            help: "Inferred from ankle motion, not measured: swing is moving, settling is slowing down, planted is treated as standing still. A wrong guess is possible; this is a hypothesis, not ground truth.",
        });
        rows.push(EstimatorRow {
            name: SPEED[f],
            value: format!(
                "{} / {} m/s",
                finite(contact.speed_mps, 2),
                finite(e.contact_speed_allowance_mps[f], 2)
            ),
            help: "Computed ankle speed from the leg model, and the extra speed the contact gate currently tolerates because heading is uncertain or the headset is moving.",
        });
        rows.push(EstimatorRow {
            name: MEAS[f],
            value: format!("{} accepted / {} rejected", diag.accepted, diag.rejected),
            help: "Heading measurements from this foot that passed or failed the filter's consistency gate. Rejections mean the foot-versus-headset motion disagreed with the model; this is a self-consistency count, not an accuracy score.",
        });
        rows.push(EstimatorRow {
            name: INNOV[f],
            value: format!(
                "NIS {} · {} inflations",
                finite(diag.last_nis, 2),
                diag.inflations
            ),
            help: "Last normalised innovation (how surprising the latest measurement was relative to the model's expectation; about 2 is typical) and how many times the filter widened its uncertainty because of this foot. Diagnostic of model consistency only, not tracking accuracy.",
        });
    }
}

fn push_leg_ik_rows(rows: &mut Vec<EstimatorRow>, s: &Snapshot) {
    use tracking_core::leg_ik::LegIkStatus as K;
    let e = &s.estimator;
    let ik = &e.leg_ik;
    let status = if !s.estimator_settings.leg_ik {
        "Off"
    } else if !e.leg_ik_active {
        "Inactive"
    } else {
        match ik.status {
            K::Disabled => "Active · disabled",
            K::Active => "Active",
            K::HeldLastOutput => "Active · held last output",
            K::FkPassthrough => "Active · fell back to FK",
            K::Unavailable => "Active · unavailable",
        }
    };
    rows.push(EstimatorRow {
        name: "Contact-aware legs",
        value: status.to_owned(),
        help: "Whether leg IK produced the published leg and hip positions this frame. Positions are inferred; this reports the solver's state, not accuracy.",
    });
    const LOCK: [&str; 2] = ["L foot lock", "R foot lock"];
    for f in 0..2 {
        rows.push(EstimatorRow {
            name: LOCK[f],
            value: format!(
                "{} · error {} cm · flat {}",
                if ik.foot_locked[f] { "locked" } else { "free" },
                finite(ik.lock_error_m[f] * 100., 1),
                finite(ik.flat_weight[f], 2)
            ),
            help: "Whether the ankle is held on its planted anchor, how far the leg chain says the ankle would be from that anchor (cm), and the flat-foot blend (0 follows the calf, 1 fully flat). The error is the model's own inconsistency, not tracking accuracy.",
        });
    }
    let c = ik.root_correction;
    let magnitude = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
    rows.push(EstimatorRow {
        name: "Hip root correction",
        value: format!("{} cm", finite(magnitude * 100., 1)),
        help: "How far leg IK moved the hips to stay consistent with the planted feet. A large value means the leg chain and the foot anchors disagree; it is not a measured error.",
    });
}

/// Heading correction of largest magnitude in degrees; `None` if none is finite.
fn max_heading_correction_degrees(s: &Snapshot) -> Option<f32> {
    s.estimator
        .stance
        .corrections
        .iter()
        .filter(|c| c.is_finite())
        .map(|c| c.to_degrees().abs())
        .max_by(f32::total_cmp)
}

/// One compact line for the estimator section in the studio footer.
fn estimator_summary(s: &Snapshot) -> String {
    let latency = s.packet_to_solve_p95_ms.map_or_else(
        || "no samples".to_owned(),
        |p95| format!("{} ms", finite(p95, 1)),
    );
    let planted = s.estimator.contacts.iter().filter(|c| c.planted()).count();
    let largest = max_heading_correction_degrees(s)
        .map_or_else(|| "n/a".to_owned(), |deg| format!("{deg:.1}°"));
    format!(
        "{} Hz · {} us solve · packet to solve p95 {latency} · prediction {} · {planted} planted · max heading correction {largest} · details in Health",
        finite(s.solve_rate_hz, 0),
        finite(s.solve_micros as f32, 0),
        if !s.estimator_settings.prediction {
            "off"
        } else if s.estimator.prediction_active {
            "active"
        } else {
            "idle"
        }
    )
}

fn finite(value: f32, decimals: usize) -> String {
    if value.is_finite() {
        format!("{value:.decimals$}")
    } else {
        "n/a".to_owned()
    }
}

/// Below this the floor offset is within normal standing variation.
const FLOOR_MISMATCH_NOTICE_M: f32 = 0.02;

/// Advice after a live neutral capture whose headset height disagrees with
/// the body model; `None` when absent, small or nonfinite.
fn floor_mismatch_text(mismatch_m: Option<f32>) -> Option<String> {
    let mismatch = mismatch_m.filter(|m| m.is_finite() && m.abs() > FLOOR_MISMATCH_NOTICE_M)?;
    let cm = mismatch.abs() * 100.;
    let (relation, effect) = if mismatch > 0. {
        ("shorter", "feet will float above the floor")
    } else {
        ("taller", "feet will sink into the floor")
    };
    Some(format!(
        "Body model is {cm:.1} cm {relation} than your headset height at the last neutral capture: {effect}. Use 'Use current headset height' or adjust eye height in step 03, then capture neutral again. If SteamVR's floor level is wrong, fix the floor instead."
    ))
}

fn assignment_orientation_status(ui: &mut egui::Ui, tracker: &TrackerView) {
    let (status, details, color) = if tracker.transport_invalidated {
        ("Awaiting new rotation".to_owned(), "Earlier queued data was discarded after a receive backlog. Move this tracker gently to send a new rotation.".to_owned(), AMBER)
    } else if tracker.estimated_held {
        (format!("Firmware-held · {:.1} s old", tracker.age_seconds), "Firmware 0.7.2 may suppress unchanged rotations. This uses the last received orientation under a continuity assumption; it does not prove current orientation or sensor health. Confidence is capped at 45%.".to_owned(), AMBER)
    } else {
        (
            format!("Measured orientation · {:.1} s old", tracker.age_seconds),
            format!(
                "Last received orientation sample: {:.2} seconds ago. Device connection age is reported separately in Health.",
                tracker.age_seconds
            ),
            MUTED,
        )
    };
    // Reserve the same single line for every state. Fresh/held transitions must
    // not move the controls below the six tracker cards while someone clicks.
    let (rect, response) =
        ui.allocate_exact_size(egui::vec2(ui.available_width(), 18.), Sense::hover());
    ui.painter()
        .with_clip_rect(rect.intersect(ui.clip_rect()))
        .text(
            rect.left_center(),
            egui::Align2::LEFT_CENTER,
            &status,
            FontId::proportional(12.),
            color,
        );
    response.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Label, ui.is_enabled(), &status));
    response.on_hover_text(details);
}

fn connected(tracker: &TrackerView) -> bool {
    tracker
        .device_packet_age_seconds
        .is_some_and(|age| age.is_finite() && age < 1.5)
}

fn calibration_active(snapshot: &Snapshot) -> bool {
    matches!(
        snapshot.calibration_request.phase,
        CalibrationRequestPhase::Countdown
            | CalibrationRequestPhase::WaitingStable
            | CalibrationRequestPhase::Moving
    )
}

fn body_role_label(role: &str) -> &'static str {
    ROLES
        .iter()
        .find(|(id, _)| *id == role)
        .map_or("selected location", |(_, label)| *label)
}

fn identification_observations(snapshot: &Snapshot) -> Vec<identification::Observation<'_>> {
    snapshot
        .trackers
        .iter()
        .map(|tracker| identification::Observation {
            id: &tracker.id,
            quaternion_xyzw: tracker.orientation,
            rotation_packets: tracker.packets,
            rotation_age_seconds: tracker.age_seconds,
            device_packet_age_seconds: if tracker.transport_invalidated {
                None
            } else {
                tracker.device_packet_age_seconds
            },
        })
        .collect()
}

fn activity_meter(ui: &mut egui::Ui, fraction: f32, label: &str, color: Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(ui.available_width(), 25.), Sense::hover());
    let fraction = if fraction.is_finite() {
        fraction.clamp(0., 1.)
    } else {
        0.
    };
    ui.painter().rect_filled(rect, 4, CARD);
    let filled = egui::Rect {
        min: rect.min,
        max: egui::pos2(rect.min.x + rect.width() * fraction, rect.max.y),
    };
    ui.painter()
        .rect_filled(filled, 4, color.gamma_multiply(0.23));
    ui.painter().text(
        rect.center(),
        egui::Align2::CENTER_CENTER,
        label,
        FontId::proportional(11.),
        TEXT,
    );
}

fn short_tracker_id(id: &str) -> String {
    let mac = id.split('/').next().unwrap_or(id);
    let mut characters: Vec<char> = mac.chars().rev().take(8).collect();
    characters.reverse();
    characters.into_iter().collect()
}

#[derive(Clone)]
struct Camera {
    yaw: f32,
    pitch: f32,
    distance: f32,
    target: Vec3,
}
impl Default for Camera {
    fn default() -> Self {
        Self {
            yaw: std::f32::consts::PI + 0.65,
            pitch: 0.2,
            distance: 3.8,
            target: Vec3::new(0., 0.9, 0.),
        }
    }
}
impl Camera {
    fn basis(&self) -> (Vec3, Vec3, Vec3, Vec3) {
        let eye = self.target
            + Vec3::new(
                self.yaw.sin() * self.pitch.cos(),
                self.pitch.sin(),
                self.yaw.cos() * self.pitch.cos(),
            ) * self.distance;
        let forward = (self.target - eye).normalize();
        let right = forward.cross(Vec3::Y).normalize();
        let up = right.cross(forward);
        (eye, forward, right, up)
    }
    fn project(&self, point: Vec3, rect: &egui::Rect) -> Option<egui::Pos2> {
        let (eye, forward, right, up) = self.basis();
        let delta = point - eye;
        let depth = delta.dot(forward);
        if !depth.is_finite() || depth < 0.1 {
            return None;
        }
        let focal = rect.height() * 1.2;
        Some(
            rect.center()
                + egui::vec2(
                    delta.dot(right) * focal / depth,
                    -delta.dot(up) * focal / depth,
                ),
        )
    }
    fn interact(&mut self, ui: &egui::Ui, response: &egui::Response) {
        let delta = ui.input(|i| i.pointer.delta());
        if response.dragged_by(egui::PointerButton::Primary) {
            self.yaw -= delta.x * 0.008;
            self.pitch = (self.pitch + delta.y * 0.006).clamp(-0.25, 1.3);
        }
        if response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
        {
            let (_, _, right, up) = self.basis();
            let scale = self.distance * 0.0018;
            self.target += (-right * delta.x + up * delta.y) * scale;
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            self.distance = (self.distance * (-scroll * 0.002).exp()).clamp(1.1, 12.);
        }
        if response.double_clicked() {
            *self = Self::default();
        }
    }
}

fn draw_scene(
    ui: &egui::Ui,
    rect: &egui::Rect,
    camera: &Camera,
    s: &Snapshot,
    show_axes: bool,
    manual_role: &str,
) {
    let p = ui.painter().with_clip_rect(*rect);
    p.rect_filled(*rect, 8, Color32::from_rgb(13, 23, 34));
    let line3 = |a: Vec3, b: Vec3, width: f32, color: Color32| {
        if let (Some(a), Some(b)) = (camera.project(a, rect), camera.project(b, rect)) {
            p.line_segment([a, b], Stroke::new(width, color));
        }
    };
    for i in -10..=10 {
        let t = i as f32 * 0.25;
        let color = if i % 4 == 0 {
            Color32::from_rgb(40, 62, 77)
        } else {
            Color32::from_rgb(25, 42, 56)
        };
        line3(Vec3::new(t, 0., -2.5), Vec3::new(t, 0., 2.5), 1., color);
        line3(Vec3::new(-2.5, 0., t), Vec3::new(2.5, 0., t), 1., color);
    }
    draw_facing_cue(&p, rect, camera, s);
    line3(
        Vec3::ZERO,
        Vec3::X * 0.45,
        2.,
        Color32::from_rgb(208, 93, 109),
    );
    line3(Vec3::ZERO, Vec3::Y * 0.45, 2., TEAL);
    line3(
        Vec3::ZERO,
        -Vec3::Z * 0.45,
        2.,
        Color32::from_rgb(97, 156, 245),
    );
    if let Some(origin) = camera.project(Vec3::ZERO, rect) {
        p.circle_filled(origin, 3., MUTED);
    }
    let label = if !s.calibrated {
        "RAW SENSOR VIEW"
    } else if s.calibration_preview {
        "REFERENCE CALIBRATION"
    } else {
        "ESTIMATED BODY POSE"
    };
    p.text(
        rect.left_top() + egui::vec2(16., 16.),
        egui::Align2::LEFT_TOP,
        label,
        FontId::proportional(11.),
        MUTED,
    );
    if let Some(age) = s
        .trackers
        .iter()
        .filter(|tracker| tracker.estimated_held)
        .map(|tracker| tracker.age_seconds)
        .max_by(f32::total_cmp)
    {
        p.text(
            rect.left_top() + egui::vec2(16., 82.),
            egui::Align2::LEFT_TOP,
            format!(
                "Firmware-held orientation · raw sample age up to {age:.1} s · confidence ≤45%"
            ),
            FontId::proportional(11.),
            AMBER,
        );
    }
    if s.calibrated && !s.joints.is_empty() {
        let mut bones = s.bones.clone();
        let (eye, forward, _, _) = camera.basis();
        bones.sort_by(|a, b| {
            let depth = |bone: &[usize; 2]| -> f32 {
                let Some(j0) = s.joints.get(bone[0]) else {
                    return 0.;
                };
                let Some(j1) = s.joints.get(bone[1]) else {
                    return 0.;
                };
                ((Vec3::from_array(j0.position) + Vec3::from_array(j1.position)) * 0.5 - eye)
                    .dot(forward)
            };
            depth(b).total_cmp(&depth(a))
        });
        for [a, b] in bones {
            if let (Some(a), Some(b)) = (s.joints.get(a), s.joints.get(b)) {
                let a = Vec3::from_array(a.position);
                let b = Vec3::from_array(b.position);
                if a.is_finite() && b.is_finite() {
                    line3(a, b, 11., Color32::from_rgb(19, 57, 64));
                    line3(a, b, 5., if s.calibration_preview { MUTED } else { TEAL });
                }
            }
        }
        let live_contacts = !s.calibration_preview && !s.desktop_session;
        for joint in &s.joints {
            let position = Vec3::from_array(joint.position);
            if !position.is_finite() {
                continue;
            }
            if let Some(at) = camera.project(position, rect) {
                let foot = match joint.name.as_str() {
                    "LeftAnkle" => Some(0),
                    "RightAnkle" => Some(1),
                    _ => None,
                };
                // Contact is a hypothesis; the tint marks it, it proves nothing.
                let tint = foot.filter(|_| live_contacts).and_then(|f| {
                    if s.estimator.leg_ik.foot_locked[f] {
                        Some(LOCKED_TINT)
                    } else if s.estimator.contacts[f].planted() {
                        Some(PLANTED_TINT)
                    } else {
                        None
                    }
                });
                p.circle_filled(at, 6., tint.unwrap_or(Color32::from_rgb(21, 40, 52)));
                p.circle_stroke(
                    at,
                    5.,
                    Stroke::new(1.5, if s.calibration_preview { MUTED } else { TEXT }),
                );
            }
        }
        if live_contacts {
            let origin = rect.left_top() + egui::vec2(16., 34.);
            for (i, (color, text)) in [
                (PLANTED_TINT, "ankle: foot planted (inferred)"),
                (LOCKED_TINT, "ankle: held by leg IK"),
            ]
            .into_iter()
            .enumerate()
            {
                let at = origin + egui::vec2(4., 6. + 14. * i as f32);
                p.circle_filled(at, 4., color);
                p.text(
                    at + egui::vec2(10., 0.),
                    egui::Align2::LEFT_CENTER,
                    text,
                    FontId::proportional(10.),
                    MUTED,
                );
            }
        }
        for target in &s.targets {
            let position = Vec3::from_array(target.position);
            if !position.is_finite() {
                continue;
            }
            if let Some(at) = camera.project(position, rect) {
                let color = if target.valid { TEAL } else { AMBER };
                p.circle_stroke(at, 10., Stroke::new(1.5, color));
                p.text(
                    at + egui::vec2(15., -4.),
                    egui::Align2::LEFT_CENTER,
                    &target.name,
                    FontId::proportional(11.),
                    color,
                );
            }
            if show_axes {
                draw_axes(&p, camera, rect, position, target.orientation, 0.13);
            }
        }
    } else {
        // A labeled scale reference only. It is never claimed as a solved pose.
        let height = if s.height_m > 0.5 { s.height_m } else { 1.65 };
        let points = [
            Vec3::new(0., height, 0.),
            Vec3::new(0., height * 0.83, 0.),
            Vec3::new(0., height * 0.57, 0.),
            Vec3::new(-0.13, height * 0.57, 0.),
            Vec3::new(-0.13, height * 0.29, 0.),
            Vec3::new(-0.13, 0.07, 0.),
            Vec3::new(0.13, height * 0.57, 0.),
            Vec3::new(0.13, height * 0.29, 0.),
            Vec3::new(0.13, 0.07, 0.),
        ];
        for (a, b) in [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 4),
            (4, 5),
            (2, 6),
            (6, 7),
            (7, 8),
        ] {
            line3(points[a], points[b], 2., Color32::from_rgb(56, 73, 88));
        }
        if let Some(at) = camera.project(Vec3::new(0., height + 0.15, 0.), rect) {
            p.text(
                at,
                egui::Align2::CENTER_CENTER,
                "Scale reference · not calibrated",
                FontId::proportional(12.),
                MUTED,
            );
        }
        for (index, tracker) in s.trackers.iter().enumerate() {
            let side = if index % 2 == 0 { -1. } else { 1. };
            let row = (index / 2) as f32;
            let position = Vec3::new(side * 0.65, 1.55 - row * 0.45, 0.);
            draw_axes(&p, camera, rect, position, tracker.orientation, 0.18);
            if let Some(at) = camera.project(position + Vec3::new(0., -0.12, 0.), rect) {
                p.text(
                    at,
                    egui::Align2::CENTER_TOP,
                    if tracker.estimated_held {
                        format!(
                            "{}\nheld {:.1} s",
                            short_tracker_id(&tracker.id),
                            tracker.age_seconds
                        )
                    } else {
                        short_tracker_id(&tracker.id)
                    },
                    FontId::proportional(11.),
                    if tracker.estimated_held {
                        AMBER
                    } else if connected(tracker) {
                        TEXT
                    } else {
                        AMBER
                    },
                );
            }
        }
        if s.trackers.is_empty() {
            p.text(
                rect.center() + egui::vec2(0., 115.),
                egui::Align2::CENTER_CENTER,
                "Waiting for tracker orientations",
                FontId::proportional(16.),
                MUTED,
            );
        }
    }
    if s.calibrated {
        let joint_name = match manual_role {
            "chest" => "Chest",
            "hip" => "Hip",
            "left_thigh" => "LeftHip",
            "right_thigh" => "RightHip",
            "left_calf" => "LeftKnee",
            "right_calf" => "RightKnee",
            "left_foot" => "LeftAnkle",
            "right_foot" => "RightAnkle",
            "left_upper_arm" => "LeftShoulder",
            "right_upper_arm" => "RightShoulder",
            _ => "",
        };
        if let (Some(joint), Some(tracker)) = (
            s.joints.iter().find(|j| j.name == joint_name),
            s.trackers
                .iter()
                .find(|t| t.role.as_deref() == Some(manual_role)),
        ) {
            let position = Vec3::from_array(joint.position);
            if position.is_finite() {
                draw_axes(&p, camera, rect, position, tracker.orientation, 0.28);
                if let Some(at) = camera.project(position, rect) {
                    p.circle_stroke(at, 14., Stroke::new(2., AMBER));
                    p.text(
                        at + egui::vec2(16., 14.),
                        egui::Align2::LEFT_TOP,
                        format!(
                            "Selected {} · raw axes ({:.1} s old)",
                            body_role_label(manual_role),
                            tracker.age_seconds
                        ),
                        FontId::proportional(11.),
                        AMBER,
                    );
                }
            }
        }
    }
    let bottom = rect.right_bottom() - egui::vec2(14., 14.);
    p.text(
        bottom,
        egui::Align2::RIGHT_BOTTOM,
        if s.calibrated {
            "Skeleton preview · positions estimated"
        } else {
            "Axes: X red · Y green · Z blue"
        },
        FontId::proportional(10.),
        MUTED,
    );
}

// Desktop forward is a fixed coordinate reference. Live heading comes only from
// a fresh headset orientation, never from the standing-space -Z axis.
fn preview_facing(s: &Snapshot) -> Option<(Vec3, &'static str)> {
    if s.desktop_session {
        Some((-Vec3::Z, "Preview forward"))
    } else {
        let (direction, label) = if let Some(direction) = s.calibration_forward {
            (direction, "Calibration forward")
        } else {
            (s.hmd_forward?, "Headset facing")
        };
        let direction = Vec3::from_array(direction);
        let horizontal = Vec3::new(direction.x, 0., direction.z);
        if !horizontal.is_finite() || horizontal.length_squared() < 1e-6 {
            return None;
        }
        Some((horizontal.normalize(), label))
    }
}

fn draw_facing_cue(p: &egui::Painter, rect: &egui::Rect, camera: &Camera, s: &Snapshot) {
    let Some((direction, label)) = preview_facing(s) else {
        p.text(
            rect.right_top() + egui::vec2(-16., 16.),
            egui::Align2::RIGHT_TOP,
            "Headset facing unavailable",
            FontId::proportional(12.),
            AMBER,
        );
        return;
    };
    let anchor = if s.desktop_session {
        Vec3::ZERO
    } else {
        s.joints
            .iter()
            .find(|j| j.name == "Hip")
            .map(|j| Vec3::from_array(j.position))
            .filter(|v| v.is_finite())
            .map_or(Vec3::ZERO, |v| Vec3::new(v.x, 0., v.z))
    };
    // The physical arrow lies on the floor and therefore follows camera orbit.
    let start = anchor + direction * 0.25 + Vec3::Y * 0.015;
    let tip = anchor + direction * 1.15 + Vec3::Y * 0.015;
    let side = Vec3::new(-direction.z, 0., direction.x);
    let wing = tip - direction * 0.29;
    if let (Some(a), Some(b), Some(left), Some(right)) = (
        camera.project(start, rect),
        camera.project(tip, rect),
        camera.project(wing + side * 0.19, rect),
        camera.project(wing - side * 0.19, rect),
    ) {
        p.line_segment([a, b], Stroke::new(9., BG));
        p.line_segment([a, b], Stroke::new(5., AMBER));
        p.add(egui::Shape::convex_polygon(
            vec![b, left, right],
            AMBER,
            Stroke::new(1.5, BG),
        ));
    }

    // A camera-relative compass remains readable when the floor arrow is edge-on
    // or has moved outside the viewport. Up points away from the camera.
    let center = rect.right_top() + egui::vec2(-41., 46.);
    let (_, forward, right, _) = camera.basis();
    let horizontal_forward = Vec3::new(forward.x, 0., forward.z).normalize_or_zero();
    let screen_direction =
        egui::vec2(direction.dot(right), -direction.dot(horizontal_forward)).normalized();
    p.circle_filled(center, 26., Color32::from_rgb(19, 28, 40));
    p.circle_stroke(center, 26., Stroke::new(1., LINE));
    let end = center + screen_direction * 18.;
    let base = center - screen_direction * 13.;
    p.arrow(base, end - base, Stroke::new(3., AMBER));
    p.text(
        center + egui::vec2(-36., -5.),
        egui::Align2::RIGHT_CENTER,
        label,
        FontId::proportional(13.),
        AMBER,
    );
    p.text(
        center + egui::vec2(-36., 12.),
        egui::Align2::RIGHT_CENTER,
        "camera-relative",
        FontId::proportional(10.),
        MUTED,
    );
}

fn draw_axes(
    p: &egui::Painter,
    camera: &Camera,
    rect: &egui::Rect,
    origin: Vec3,
    q: [f32; 4],
    length: f32,
) {
    if !q.iter().all(|v| v.is_finite()) || !origin.is_finite() {
        return;
    }
    let q = Quat::from_array(q);
    if q.length_squared() < 1e-8 {
        return;
    }
    let q = q.normalize();
    for (axis, color) in [
        (Vec3::X, Color32::from_rgb(240, 105, 120)),
        (Vec3::Y, TEAL),
        (Vec3::Z, Color32::from_rgb(101, 162, 255)),
    ] {
        if let (Some(a), Some(b)) = (
            camera.project(origin, rect),
            camera.project(origin + q * axis * length, rect),
        ) {
            p.line_segment([a, b], Stroke::new(2.5, color));
            p.circle_filled(b, 3., color);
        }
    }
}

#[cfg(test)]
mod identification_ui_tests {
    use super::*;

    #[test]
    fn facing_cue_distinguishes_preview_live_and_latched_heading() {
        let mut snapshot = Snapshot {
            desktop_session: true,
            ..Default::default()
        };
        assert_eq!(
            preview_facing(&snapshot),
            Some((-Vec3::Z, "Preview forward"))
        );
        snapshot.desktop_session = false;
        assert_eq!(preview_facing(&snapshot), None);
        snapshot.hmd_forward = Some([2., 1., 0.]);
        assert_eq!(preview_facing(&snapshot), Some((Vec3::X, "Headset facing")));
        snapshot.calibration_forward = Some([0., 0., 3.]);
        assert_eq!(
            preview_facing(&snapshot),
            Some((Vec3::Z, "Calibration forward"))
        );
        snapshot.calibration_forward = Some([f32::NAN, 0., 0.]);
        assert_eq!(preview_facing(&snapshot), None);
    }

    #[test]
    fn floor_mismatch_notice_needs_a_finite_offset_beyond_two_centimetres() {
        for quiet in [
            None,
            Some(0.),
            Some(0.019),
            Some(-0.02),
            Some(f32::NAN),
            Some(f32::INFINITY),
        ] {
            assert_eq!(floor_mismatch_text(quiet), None, "{quiet:?}");
        }
        let float = floor_mismatch_text(Some(0.042)).unwrap();
        assert!(float.contains("4.2 cm shorter") && float.contains("float"));
        let sink = floor_mismatch_text(Some(-0.05)).unwrap();
        assert!(sink.contains("5.0 cm taller") && sink.contains("sink"));
    }

    #[test]
    fn estimator_rows_never_print_nonfinite_values_or_invent_latency() {
        let mut snapshot = Snapshot {
            solve_rate_hz: f32::NAN,
            solve_micros: f64::INFINITY,
            ..Default::default()
        };
        snapshot.estimator.hmd_extrapolated_secs = f32::NAN;
        snapshot.estimator.standing_space_rebases = 2;
        snapshot.estimator.last_rebase_degrees = f32::NEG_INFINITY;
        let rows = estimator_rows(&snapshot);
        for row in &rows {
            assert!(
                !row.value.contains("NaN") && !row.value.replace("inflations", "").contains("inf"),
                "{}",
                row.value
            );
        }
        let latency = rows
            .iter()
            .find(|row| row.name.starts_with("Packet"))
            .unwrap();
        assert_eq!(latency.value, "no samples");
        assert!(!estimator_summary(&snapshot).contains("NaN"));
        snapshot.estimator_settings.prediction = false;
        snapshot.estimator.prediction_active = true;
        let prediction = estimator_rows(&snapshot)
            .into_iter()
            .find(|row| row.name == "Motion prediction")
            .unwrap();
        assert_eq!(prediction.value, "Off");
    }

    #[test]
    fn estimator_rows_show_heading_in_degrees_and_never_print_nonfinite() {
        use tracking_core::contact::ContactPhase;
        let mut snapshot = Snapshot::default();
        snapshot.estimator.stance_active = true;
        snapshot.estimator.stance.corrections[2] = 0.05934119; // 3.4 deg
        snapshot.estimator.stance.sigma[2] = 0.020943952; // 1.2 deg
        snapshot.estimator.contacts[0].phase = ContactPhase::Planted;
        snapshot.estimator.contacts[1].phase = ContactPhase::Settling;
        snapshot.estimator.leg_ik.lock_error_m[0] = 0.013;
        let rows = estimator_rows(&snapshot);
        let thigh = rows
            .iter()
            .find(|r| r.name == "Heading correction · L thigh")
            .unwrap();
        assert_eq!(thigh.value, "+3.4° ± 1.2°");
        assert_eq!(
            rows.iter()
                .find(|r| r.name == "L foot contact")
                .unwrap()
                .value,
            "planted"
        );
        assert_eq!(
            rows.iter()
                .find(|r| r.name == "R foot contact")
                .unwrap()
                .value,
            "settling"
        );
        assert!(
            rows.iter()
                .find(|r| r.name == "L foot lock")
                .unwrap()
                .value
                .contains("error 1.3 cm")
        );
        assert!(estimator_summary(&snapshot).contains("1 planted"));
        assert!(estimator_summary(&snapshot).contains("3.4°"));

        snapshot.estimator.stance.corrections = [f32::NAN; 6];
        snapshot.estimator.stance.sigma = [f32::INFINITY; 6];
        snapshot.estimator.stance.feet[0].last_nis = f32::NAN;
        snapshot.estimator.contacts[0].speed_mps = f32::INFINITY;
        snapshot.estimator.contact_speed_allowance_mps = [f32::NAN; 2];
        snapshot.estimator.leg_ik.lock_error_m = [f32::NAN; 2];
        snapshot.estimator.leg_ik.root_correction = [f32::INFINITY, 0., 0.];
        snapshot.estimator.leg_ik.flat_weight = [f32::NAN; 2];
        for row in estimator_rows(&snapshot) {
            assert!(
                !row.value.contains("NaN") && !row.value.replace("inflations", "").contains("inf"),
                "{}: {}",
                row.name,
                row.value
            );
            assert!(!row.help.is_empty());
        }
        let summary = estimator_summary(&snapshot);
        assert!(!summary.contains("NaN") && !summary.contains("inf"));
        assert!(summary.contains("max heading correction n/a"));
    }

    #[test]
    fn behind_camera_places_facing_to_the_away_side() {
        for direction in [-Vec3::Z, Vec3::X, Vec3::Z, -Vec3::X] {
            let front_yaw = direction.x.atan2(direction.z);
            let camera = Camera {
                yaw: front_yaw + std::f32::consts::PI,
                ..Camera::default()
            };
            let (_, forward, _, _) = camera.basis();
            assert!(forward.dot(direction) > 0.9);
        }
    }

    #[test]
    fn queued_transport_rotations_cannot_identify_a_tracker_but_new_clean_motion_can() {
        let mut snapshot = Snapshot {
            trackers: vec![TrackerView {
                id: "moving".to_owned(),
                orientation: [0., 0., 0., 1.],
                packets: 100,
                age_seconds: 0.,
                device_packet_age_seconds: Some(0.),
                transport_invalidated: true,
                ..Default::default()
            }],
            ..Default::default()
        };
        let mut session =
            identification::Session::start(0., &identification_observations(&snapshot));
        for step in 1..=90 {
            snapshot.trackers[0].packets += 1;
            snapshot.trackers[0].orientation = Quat::from_rotation_y(step as f32 * 0.04).to_array();
            session.observe(
                f64::from(step) / 30.,
                &identification_observations(&snapshot),
            );
        }
        assert_eq!(
            session.outcome(),
            Some(&identification::Outcome::NoMovement)
        );
        snapshot.trackers[0].transport_invalidated = false;
        let mut clean = identification::Session::start(3., &identification_observations(&snapshot));
        for step in 1..=90 {
            snapshot.trackers[0].packets += 1;
            snapshot.trackers[0].orientation = Quat::from_rotation_y(step as f32 * 0.04).to_array();
            clean.observe(
                3. + f64::from(step) / 30.,
                &identification_observations(&snapshot),
            );
        }
        assert_eq!(clean.candidate_id(), Some("moving"));
        snapshot.trackers[0].transport_invalidated = true;
        assert!(!clean.candidate_still_connected(&identification_observations(&snapshot)));
    }
}
