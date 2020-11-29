pub use crate::{madgwick::Madgwick};
pub use crate::mahony::Mahony;
use nalgebra::{RealField, Vector3, UnitQuaternion};

mod madgwick;
mod mahony;

#[derive(Debug)]
pub enum AhrsError {
	NormalizationError,
}

pub trait Ahrs<N: RealField> where Self: Send + Sync + 'static {
	fn update_imu(&mut self,
				  gyroscope: &Vector3<N>,
				  accelerometer: &Vector3<N>,
				  dt: N) -> Result<(), AhrsError>;
	fn orientation(&self) -> UnitQuaternion<N>;
}