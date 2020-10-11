// Adapted from `https://github.com/jsjolund/f4/blob/master/src/madgwick_ahrs.rs`

use autopilot::ImuData;
use nalgebra::{UnitQuaternion, Quaternion};
use num_traits::identities::Zero;

pub struct MadgwickAhrs {
	q: Quaternion<f32>,
	beta: f32, // 2 * proportional gain (Kp)
}

impl MadgwickAhrs {
	pub fn new(beta: f32) -> Self {
		MadgwickAhrs {
			q: Quaternion::zero(),
			beta, // 2 * proportional gain
		}
	}

	pub fn update(
		&mut self,
		imu_data: ImuData,
		dt: f32,
	) -> UnitQuaternion<f32> {
		let a = imu_data.acc;
		let g = imu_data.gyr;
		let m = imu_data.mag;

		let q = self.q;

		// Rate of change of quaternion from gyroscope
		let mut q_dot = Quaternion::new(
			0.5 * (q.i * g.z + q.j * g.y - q.k * g.x),
			0.5 * (-q.j * g.x - q.k * g.y - q.w * g.z),
			0.5 * (q.i * g.x + q.k * g.z - q.w * g.y),
			0.5 * (q.i * g.y - q.j * g.z + q.w * g.x),
		);

		// Compute feedback only if accelerometer measurement valid
		// (avoids NaN in accelerometer normalisation)
		if !((a.x.is_zero()) && (a.y.is_zero()) && (a.z.is_zero())) {
			// Normalise accelerometer and magnetometer measurement
			let a = a.normalize();
			let m = m.normalize();

			// Auxiliary variables to avoid repeated arithmetic
			let _2qxmx = 2.0 * q.i * m.x;
			let _2qxmy = 2.0 * q.i * m.y;
			let _2qxmz = 2.0 * q.i * m.z;
			let _2qymx = 2.0 * q.j * m.x;
			let _2qx = 2.0 * q.i;
			let _2qy = 2.0 * q.j;
			let _2qz = 2.0 * q.k;
			let _2qw = 2.0 * q.w;
			let _2qxqz = 2.0 * q.i * q.k;
			let _2qxqw = 2.0 * q.k * q.w;
			let qxqx = q.i * q.i;
			let qxqy = q.i * q.j;
			let qxqz = q.i * q.k;
			let qxqw = q.i * q.w;
			let qyqy = q.j * q.j;
			let qyqz = q.j * q.k;
			let qyqw = q.j * q.w;
			let qzqz = q.k * q.k;
			let qzqw = q.k * q.w;
			let qwqw = q.w * q.w;

			// Reference direction of Earth's magnetic field
			let hx = m.x * qxqx - _2qxmy * q.w + _2qxmz * q.k + m.x * qyqy + _2qy * m.y * q.k
				+ _2qy * m.z * q.w - m.x * qzqz - m.x * qwqw;
			let hy = _2qxmx * q.w + m.y * qxqx - _2qxmz * q.j + _2qymx * q.k - m.y * qyqy
				+ m.y * qzqz + _2qz * m.z * q.w - m.y * qwqw;
			let _2bx = (hx * hx + hy * hy).sqrt();
			let _2bz = -_2qxmx * q.k + _2qxmy * q.j + m.z * qxqx + _2qymx * q.w - m.z * qyqy
				+ _2qz * m.y * q.w - m.z * qzqz + m.z * qwqw;
			let _4bx = 2.0 * _2bx;
			let _4bz = 2.0 * _2bz;

			// Gradient decent algorithm corrective step
			let s = Quaternion::<f32>::new(
				_2qy * (2.0 * qyqw - _2qxqz - a.x) + _2qz * (2.0 * qxqy + _2qxqw - a.y)
					+ (-_4bx * q.w + _2bz * q.j)
					* (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
					+ (-_2bx * q.i + _2bz * q.k)
					* (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
					+ _2bx * q.j * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
				-_2qz * (2.0 * qyqw - _2qxqz - a.x) + _2qy * (2.0 * qxqy + _2qxqw - a.y)
					- _2bz * q.k * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
					+ (-_2bx * q.w + _2bz * q.j)
					* (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
					+ _2bx * q.k * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
				_2qw * (2.0 * qyqw - _2qxqz - a.x) + _2qx * (2.0 * qxqy + _2qxqw - a.y)
					- 4.0 * q.j * (1.0 - 2.0 * qyqy - 2.0 * qzqz - a.z)
					+ _2bz * q.w * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
					+ (_2bx * q.k + _2bz * q.i)
					* (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
					+ (_2bx * q.w - _4bz * q.j)
					* (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
				-_2qx * (2.0 * qyqw - _2qxqz - a.x) + _2qw * (2.0 * qxqy + _2qxqw - a.y)
					- 4.0 * q.k * (1.0 - 2.0 * qyqy - 2.0 * qzqz - a.z)
					+ (-_4bx * q.k - _2bz * q.i)
					* (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
					+ (_2bx * q.j + _2bz * q.w)
					* (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
					+ (_2bx * q.i - _4bz * q.k)
					* (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
			);

			// Apply feedback step
			q_dot = q_dot - self.beta * s.normalize();
		}

		// Integrate rate of change of quaternion to yield quaternion
		self.q = q + (dt * q_dot).normalize();
		UnitQuaternion::from_quaternion(self.q.clone())
	}
}