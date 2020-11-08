use nalgebra::{Quaternion, Vector3, UnitQuaternion, RealField};
use crate::{AhrsError, Ahrs};

#[derive(Debug)]
pub struct Mahony<N: RealField> {
	/// Proportional filter gain constant.
	kp: N,
	/// Integral filter gain constant.
	ki: N,
	/// Integral error vector.
	e_int: Vector3<N>,
	/// Filter state quaternion.
	quat: Quaternion<N>,
}

impl<N: RealField> Mahony<N> {
	pub fn new(kp: N, ki: N) -> Self {
		Mahony {
			kp,
			ki,
			e_int: nalgebra::zero(),
			quat: Quaternion::from_parts(N::one(), nalgebra::zero::<nalgebra::Vector3<N>>()),
		}
	}
}
	// fn update(
	// 	&mut self,
	// 	gyroscope: &Vector3<N>,
	// 	accelerometer: &Vector3<N>,
	// 	magnetometer: &Vector3<N>,
	// ) -> Result<&Quaternion<N>, &str> {
	// 	let q = self.quat;
	//
	// 	let zero: N = nalgebra::zero();
	// 	let two: N = nalgebra::convert(2.0);
	// 	let half: N = nalgebra::convert(0.5);
	//
	// 	// Normalize accelerometer measurement
	// 	let accel = match accelerometer.try_normalize(zero) {
	// 		Some(n) => n,
	// 		None => {
	// 			return Err("Accelerometer norm divided by zero.");
	// 		}
	// 	};
	//
	// 	// Normalize magnetometer measurement
	// 	let mag = match magnetometer.try_normalize(zero) {
	// 		Some(n) => n,
	// 		None => {
	// 			return Err("Magnetometer norm divided by zero.");
	// 		}
	// 	};
	//
	// 	// Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
	// 	let h = q * (Quaternion::from_parts(zero, mag) * q.conjugate());
	// 	let b = Quaternion::new(zero, Vector2::new(h[0], h[1]).norm(), zero, h[2]);
	//
	// 	#[rustfmt::skip]
	// 		let v = Vector3::new(
	// 		two*( q[0]*q[2] - q[3]*q[1] ),
	// 		two*( q[3]*q[0] + q[1]*q[2] ),
	// 		q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2]
	// 	);
	//
	// 	#[rustfmt::skip]
	// 		let w = Vector3::new(
	// 		two*b[0]*(half - q[1]*q[1] - q[2]*q[2]) + two*b[2]*(q[0]*q[2] - q[3]*q[1]),
	// 		two*b[0]*(q[0]*q[1] - q[3]*q[2])        + two*b[2]*(q[3]*q[0] + q[1]*q[2]),
	// 		two*b[0]*(q[3]*q[1] + q[0]*q[2])        + two*b[2]*(half - q[0]*q[0] - q[1]*q[1])
	// 	);
	//
	// 	let e: Vector3<N> = accel.cross(&v) + mag.cross(&w);
	//
	// 	// Error is sum of cross product between estimated direction and measured direction of fields
	// 	if self.ki > zero {
	// 		self.e_int += e * self.sample_period;
	// 	} else {
	// 		//Vector3::new(zero, zero, zero);
	// 		self.e_int.x = zero;
	// 		self.e_int.y = zero;
	// 		self.e_int.z = zero;
	// 	}
	//
	// 	// Apply feedback terms
	// 	let gyro = *gyroscope + e * self.kp + self.e_int * self.ki;
	//
	// 	// Compute rate of change of quaternion
	// 	let qDot = q * Quaternion::from_parts(zero, gyro) * half;
	//
	// 	// Integrate to yield quaternion
	// 	self.quat = (q + qDot * self.sample_period).normalize();
	//
	// 	Ok(&self.quat)
	// }

impl<N: RealField> Ahrs<N> for Mahony<N> {
	fn update_imu(
		&mut self,
		gyroscope: &Vector3<N>,
		accelerometer: &Vector3<N>,
		dt: N,
	) -> Result<(), AhrsError> {
		let q = self.quat;

		let zero: N = nalgebra::zero();
		let two: N = nalgebra::convert(2.0);
		let half: N = nalgebra::convert(0.5);

		// Normalize accelerometer measurement
		if let Some(_n) = accelerometer.try_normalize(zero) {
			#[rustfmt::skip]
				let v = Vector3::new(
				two * (q[0] * q[2] - q[3] * q[1]),
				two * (q[3] * q[0] + q[1] * q[2]),
				q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2],
			);

			let e = accelerometer.cross(&v);

			// Error is sum of cross product between estimated direction and measured direction of fields
			if self.ki > zero {
				self.e_int += e * dt;
			} else {
				self.e_int.x = zero;
				self.e_int.y = zero;
				self.e_int.z = zero;
			}

			// Apply feedback terms
			let gyro = *gyroscope + e * self.kp + self.e_int * self.ki;

			// Compute rate of change of quaternion
			let q_dot = q * Quaternion::from_parts(zero, gyro) * half;

			// Integrate to yield quaternion
			self.quat = (q + q_dot * dt).normalize();
		}
		Ok(())
	}

	fn quaternion(&self) -> UnitQuaternion<N> {
		UnitQuaternion::from_quaternion(self.quat.clone())
	}
}