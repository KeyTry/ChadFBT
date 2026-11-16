//! VRChat's OSC tracker contract uses Unity left-handed space and Z-X-Y Euler rotations.
//! This adapter is deliberately separate from the right-handed estimation core.
use glam::{EulerRot, Quat};

fn osc_string(out: &mut Vec<u8>, value: &str) {
    out.extend_from_slice(value.as_bytes());
    out.push(0);
    while !out.len().is_multiple_of(4) {
        out.push(0);
    }
}

pub fn osc_vec3(address: &str, value: [f32; 3]) -> Option<Vec<u8>> {
    if !value.iter().all(|v| v.is_finite()) || !address.starts_with('/') {
        return None;
    }
    let mut out = Vec::with_capacity(64);
    osc_string(&mut out, address);
    osc_string(&mut out, ",fff");
    for component in value {
        out.extend_from_slice(&component.to_be_bytes());
    }
    Some(out)
}

/// Reflect through XY: p' = (x,y,-z), q' = (-x,-y,z,w).
pub fn unity_pose(position: [f32; 3], xyzw: [f32; 4]) -> Option<([f32; 3], [f32; 3])> {
    if !position.iter().chain(xyzw.iter()).all(|x| x.is_finite()) {
        return None;
    }
    let q = Quat::from_xyzw(-xyzw[0], -xyzw[1], xyzw[2], xyzw[3]);
    if !(0.99..=1.01).contains(&q.length_squared()) {
        return None;
    }
    // glam intrinsic YXZ corresponds to Unity's extrinsic Z, X, Y application.
    let (y, x, z) = q.normalize().to_euler(EulerRot::YXZ);
    Some((
        [position[0], position[1], -position[2]],
        [x.to_degrees(), y.to_degrees(), z.to_degrees()],
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn osc_wire_padding_types_and_endianness() {
        let bytes = osc_vec3("/test", [1., 2., 3.]).unwrap();
        assert_eq!(&bytes[..8], b"/test\0\0\0");
        assert_eq!(&bytes[8..16], b",fff\0\0\0\0");
        assert_eq!(&bytes[16..20], &1f32.to_be_bytes());
        assert_eq!(bytes.len(), 28);
    }
    #[test]
    fn rejects_nonfinite() {
        assert!(unity_pose([f32::NAN, 0., 0.], [0., 0., 0., 1.]).is_none());
    }
    #[test]
    fn reflection_matches_basis_vectors_for_combined_rotations() {
        let q = Quat::from_euler(EulerRot::YXZ, 0.7, -0.4, 0.2);
        let (p, e) = unity_pose([1., 2., -3.], q.to_array()).unwrap();
        assert_eq!(p, [1., 2., 3.]);
        let u = Quat::from_euler(
            EulerRot::YXZ,
            e[1].to_radians(),
            e[0].to_radians(),
            e[2].to_radians(),
        );
        let reflect = |v: glam::Vec3| glam::Vec3::new(v.x, v.y, -v.z);
        for v in [glam::Vec3::X, glam::Vec3::Y, glam::Vec3::Z] {
            assert!((u * reflect(v) - reflect(q * v)).length() < 1e-5);
        }
    }
}
