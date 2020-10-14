#![allow(non_snake_case)]
#![allow(clippy::many_single_char_names)]

use nalgebra::{Matrix4, Matrix6, Quaternion, Scalar, Vector2, Vector3, Vector4, Vector6, UnitQuaternion};
use simba::simd::SimdValue;
use std::time::Instant;

#[derive(Debug)]
pub struct Madgwick<N: Scalar + SimdValue> {
	beta: N,
	quaternion: Quaternion<N>,
	last_instant: Option<Instant>,
}

impl<N: Scalar + SimdValue + num_traits::One + num_traits::Zero + simba::scalar::RealField> Madgwick<N> {

	pub fn new(beta: N) -> Self {
		Madgwick::with_quaternion(
			beta,
			Quaternion::new(N::one(), N::zero(), N::zero(), N::zero()),
		)
	}

	pub fn with_quaternion(beta: N, quaternion: Quaternion<N>) -> Self {
		Madgwick {
			beta,
			quaternion,
			last_instant: None,
		}
	}

	pub fn with_instant(beta: N, instant: Instant, quaternion: Quaternion<N>) -> Self {
		let mut instance = Self::with_quaternion(beta, quaternion);
		instance.last_instant = Some(instant);
		instance
	}
	//
	// fn update(
	// 	&mut self,
	// 	gyroscope: Vector3<N>,
	// 	accelerometer: Vector3<N>,
	// 	magnetometer: Vector3<N>,
	// 	dt: N,
	// ) -> Result<UnitQuaternion<N>, AhrsError> {
	// 	let q = &self.quat;
	//
	// 	let zero: N = nalgebra::zero();
	// 	let two: N = nalgebra::convert(2.0);
	// 	let four: N = nalgebra::convert(4.0);
	// 	let half: N = nalgebra::convert(0.5);
	//
	// 	// Normalize accelerometer measurement
	// 	let accel = match accelerometer.try_normalize(zero) {
	// 		Some(n) => n,
	// 		None => return Err(AhrsError::NullAccDataNorm),
	// 	};
	//
	// 	// Normalize magnetometer measurement
	// 	let mag = match magnetometer.try_normalize(zero) {
	// 		Some(n) => n,
	// 		None => return Err(AhrsError::NullMagDataNorm)
	// 	};
	//
	// 	// Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
	// 	let h = q * (Quaternion::from_parts(zero, mag.clone_owned()) * q.conjugate());
	// 	let b = Quaternion::new(zero, Vector2::new(h[0], h[1]).norm(), zero, h[2]);
	//
	// 	// Gradient descent algorithm corrective step
	// 	let F = Vector6::new(
	// 		two * (q[0] * q[2] - q[3] * q[1]) - accel[0],
	// 		two * (q[3] * q[0] + q[1] * q[2]) - accel[1],
	// 		two * (half - q[0] * q[0] - q[1] * q[1]) - accel[2],
	// 		two * b[0] * (half - q[1] * q[1] - q[2] * q[2]) + two * b[2] * (q[0] * q[2] - q[3] * q[1]) - mag[0],
	// 		two * b[0] * (q[0] * q[1] - q[3] * q[2]) + two * b[2] * (q[3] * q[0] + q[1] * q[2]) - mag[1],
	// 		two * b[0] * (q[3] * q[1] + q[0] * q[2]) + two * b[2] * (half - q[0] * q[0] - q[1] * q[1]) - mag[2],
	// 	);
	//
	// 	let J_t = Matrix6::new(
	// 		-two * q[1], two * q[0], zero, -two * b[2] * q[1], -two * b[0] * q[2] + two * b[2] * q[0], two * b[0] * q[1],
	// 		two * q[2], two * q[3], -four * q[0], two * b[2] * q[2], two * b[0] * q[1] + two * b[2] * q[3], two * b[0] * q[2] - four * b[2] * q[0],
	// 		-two * q[3], two * q[2], -four * q[1], -four * b[0] * q[1] - two * b[2] * q[3], two * b[0] * q[0] + two * b[2] * q[2], two * b[0] * q[3] - four * b[2] * q[1],
	// 		two * q[0], two * q[1], zero, -four * b[0] * q[2] + two * b[2] * q[0], -two * b[0] * q[3] + two * b[2] * q[1], two * b[0] * q[0],
	// 		zero, zero, zero, zero, zero, zero,
	// 		zero, zero, zero, zero, zero, zero,
	// 	);
	//
	// 	let step = (J_t * F).normalize();
	//
	// 	// Compute rate of change for quaternion
	// 	let qDot = q * Quaternion::from_parts(zero, gyroscope) * half
	// 		- Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;
	//
	// 	// Integrate to yield quaternion
	// 	self.quat = (q + qDot * dt).normalize();
	//
	// 	Ok(UnitQuaternion::from_quaternion(self.quat.clone()))
	// }

	pub fn update_imu(
		&mut self,
		gyroscope: Vector3<N>,
		accelerometer: Vector3<N>,
		instant: Instant,
	) -> UnitQuaternion<N> {
		if self.last_instant.is_some() && self.last_instant.unwrap() != instant {
			let q = &self.quaternion;

			let zero: N = nalgebra::zero();
			let two: N = nalgebra::convert(2.0);
			let four: N = nalgebra::convert(4.0);
			let half: N = nalgebra::convert(0.5);

			// Normalize accelerometer measurement
			let accel = match accelerometer.try_normalize(zero) {
				Some(n) => n,
				None => return UnitQuaternion::from_quaternion(self.quaternion.clone())
			};

			// Gradient descent algorithm corrective step
			#[rustfmt::skip]
				let F = Vector4::new(
				two * (q[0] * q[2] - q[3] * q[1]) - accel[0],
				two * (q[3] * q[0] + q[1] * q[2]) - accel[1],
				two * (half - q[0] * q[0] - q[1] * q[1]) - accel[2],
				zero,
			);

			#[rustfmt::skip]
			let J_t = Matrix4::new(
				-two * q[1], two * q[0], zero, zero,
				two * q[2], two * q[3], -four * q[0], zero,
				-two * q[3], two * q[2], -four * q[1], zero,
				two * q[0], two * q[1], zero, zero,
			);

			let step = (J_t * F).normalize();

			// Compute rate of change of quaternion
			let qDot = (q * Quaternion::from_parts(zero, gyroscope)) * half
				- Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

			// Integrate to yield quaternion
			let dt: N = nalgebra::convert((instant - self.last_instant.unwrap()).as_secs_f64());
			self.quaternion = (q + qDot * dt).normalize();
		}

		self.last_instant = Some(instant);

		UnitQuaternion::from_quaternion(self.quaternion.clone())
	}
}

