pub use crate::{madgwick::Madgwick};
pub use crate::mahony::Mahony;
use nalgebra::{RealField, Vector3, UnitQuaternion};

mod madgwick;
mod mahony;

#[derive(Debug)]
pub enum AhrsError {
	NormalizationError,
}

trait Ahrs<N: RealField> {
	fn update_imu(&mut self, gyroscope: &Vector3<N>, accelerometer: &Vector3<N>, dt: N) -> Result<UnitQuaternion<N>, AhrsError>;
}