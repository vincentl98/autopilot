use nalgebra::RealField;
use serde::{Deserialize, Serialize};
use std::ops::Sub;

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct RollPitch<N> {
	pub roll: N,
	pub pitch: N,
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct RollPitchYaw<N> {
	pub roll: N,
	pub pitch: N,
	pub yaw: N,
}

impl<N: RealField> Into<RollPitchYaw<N>> for (N, N, N) {
	fn into(self) -> RollPitchYaw<N> {
		RollPitchYaw {
			roll: self.0,
			pitch: self.1,
			yaw: self.2,
		}
	}
}

impl<N: RealField> Sub for RollPitchYaw<N> {
	type Output = Self;

	fn sub(self, rhs: Self) -> Self::Output {
		RollPitchYaw {
			roll: self.roll - rhs.roll,
			pitch: self.pitch - rhs.pitch,
			yaw: self.yaw - rhs.yaw,
		}
	}
}
