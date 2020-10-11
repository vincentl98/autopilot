use nalgebra::Vector3;
use std::time::Instant;

#[derive(Clone, Debug)]
pub struct ImuData {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
    pub mag: Vector3<f32>,
}

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub enum Input {
    // Acceleration((Vector3<f32>, Instant)),
    // AngularVelocity((Vector3<f32>, Instant)),
    // BarometricPressure((f32, Instant)),
    // MagneticField((Vector3<f32>, Instant)),
    /// Euler angles is represented as roll, pitch, yaw/heading.
    // OrientationEuler((EulerAngles<f32, ()>, Instant)),
    // OrientationQuaternion((Quaternion<f32>, Instant)),
    // RcChannels(sbus::Packet),
    Imu((ImuData, Instant)),
    SoftArmed(bool),
    // Temperature((f32, Instant)),
}
