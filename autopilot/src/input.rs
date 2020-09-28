use mint::{EulerAngles, Quaternion, Vector3};
use std::time::Instant;

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub enum Input {
    Acceleration((Vector3<f32>, Instant)),
    AngularVelocity((Vector3<f32>, Instant)),
    BarometricPressure((f32, Instant)),
    MagneticField((Vector3<f32>, Instant)),
    /// Euler angles is represented as roll, pitch, yaw/heading.
    OrientationEuler((EulerAngles<f32, ()>, Instant)),
    OrientationQuaternion((Quaternion<f32>, Instant)),
    RcChannels(sbus::Packet),
    SoftArmed(bool),
    Temperature((f32, Instant)),
}
