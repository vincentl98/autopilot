use ahrs::{Madgwick};
use approx::relative_eq;
use nalgebra::{Quaternion, Vector3, UnitQuaternion};
use std::f64;
use std::time::{Instant, Duration};

const DEFAULT_DT: u64 = 1_000_000 / 256;
const DEFAULT_DURATION: Duration = Duration::from_nanos(DEFAULT_DT);

// accel, gyro, mag values
macro_rules! default_sensors(
  () => {
    (
      Vector3::new(0.06640625, 0.9794922, -0.01269531),
      Vector3::new(68.75, 34.25, 3.0625),
      Vector3::new(0.171875, -0.4536133, -0.04101563)
    )
  };
);

// #[test]
// fn test_update_accel_zero() {
//     let mut ahrs = Madgwick::default();
//
//     let g: Vector3<f64> = Vector3::new(1.0, 1.0, 1.0);
//     let a: Vector3<f64> = Vector3::new(0.0, 0.0, 0.0);
//     let m: Vector3<f64> = Vector3::new(1.0, 1.0, 1.0);
//
//     let res = ahrs.update(g, a, m, DEFAULT_DT);
//
//     let fail_message = "Normalizing zero-value accel should have failed.";
//
//     assert!(res.is_err(), fail_message);
// }
//
// #[test]
// fn test_update_mag_zero() {
//     let mut ahrs = Madgwick::default();
//
//     let g: Vector3<f64> = Vector3::new(1.0, 1.0, 1.0);
//     let a: Vector3<f64> = Vector3::new(1.0, 1.0, 1.0);
//     let m: Vector3<f64> = Vector3::new(0.0, 0.0, 0.0);
//
//     let res = ahrs.update(g, a, m, DEFAULT_DT);
//
//     let fail_message = "Normalizing zero-value mag should have failed.";
//
//     assert!(res.is_err(), fail_message);
// }

// #[test]
// fn test_madgwick_update() {
//     let start_quat = Quaternion::new(
//         0.7252997863255918f64,
//         0.6869689552600526,
//         -0.04486780259245286,
//         0.0008687666471569602,
//     );
//
//     let mut ahrs = Madgwick::default();
//     ahrs.quat = start_quat;
//
//     let (accel, gyro, mag) = default_sensors!();
//
//     let actual = ahrs
//         .update((gyro * (f64::consts::PI / 180.0)), accel, mag, DEFAULT_DT)
//         .unwrap();
//
//
//     let expected = UnitQuaternion::from_quaternion(Quaternion::new(
//         0.7235467139148768,
//         0.6888611247479446,
//         -0.04412605927634125,
//         0.001842413287185898,
//     ));
//
//     let fail_message = format!(
//         "quaternions did not match:\n\
//         actual: {:?}\n\
//         expect: {:?}",
//         actual, expected
//     );
//
//     assert!(relative_eq!(actual, expected), fail_message);
// }

#[test]
fn test_madgwick_update_imu() {
    let start_quat = Quaternion::new(
        0.7208922848226422,
        0.6922487447935516,
        -0.01829063767755937,
        0.02777483732249482,
    );

    let now = Instant::now();

    let mut ahrs = Madgwick::with_instant(0.1, now, start_quat);

    let (accel, gyro, _) = default_sensors!();

    let actual = ahrs
        .update_imu(gyro * f64::consts::PI / 180.0, accel, now + DEFAULT_DURATION)
        .into_inner();

    let actual = UnitQuaternion::from_quaternion(actual);

    let expected = UnitQuaternion::from_quaternion(Quaternion::new(
        0.7208904870913246,
        0.6922506012721736,
        -0.018289818944577468,
        0.027775766101434042,
    ));

    let fail_message = format!(
        "quaternions did not match:\n\
      actual: {:?}\n\
      expect: {:?}",
        actual, expected
    );

    assert!(relative_eq!(actual, expected), fail_message);
}
