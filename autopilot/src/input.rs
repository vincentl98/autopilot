use nalgebra::{Vector3, UnitQuaternion, RealField, zero};
use std::time::{Instant};
use std::fmt::{Display, Formatter};
use std::fmt;

pub const G: f64 = 9.80665;

#[derive(Clone, Debug)]
pub struct ImuData<N: RealField> {
	pub acc: Vector3<N>,
	pub gyr: Vector3<N>,
	pub mag: Vector3<N>,
	pub instant: Instant,
}

impl<N: RealField> Default for ImuData<N> {
	fn default() -> Self {
		ImuData {
			acc: zero(),
			gyr: zero(),
			mag: zero(),
			instant: Instant::now(),
		}
	}
}

#[derive(Clone, Debug, Default)]
pub struct NavioAdcData<N: RealField> {
	pub board_voltage: N,
	pub servo_voltage: N,
	pub external_voltage: N,
	pub external_current: N,
	pub adc_port_2: N,
	pub adc_port_3: N,
}

impl<N: RealField> Display for NavioAdcData<N> {
	fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
		Ok(write!(
			f,
			"Ext. voltage: {:.1} V, \
			 Ext. current: {:.1} A, \
			 Board voltage: {:.1} V",
			self.external_voltage,
			self.external_current,
			self.board_voltage,
		)?)
	}
}

pub type RcChannels<N> = Option<[N; 16]>;

#[derive(Clone, Debug)]
pub enum Input<N: RealField> {
	RcChannels(RcChannels<N>),
	NavioAdc(NavioAdcData<N>),
	Orientation((UnitQuaternion<N>, Instant)),
	SoftArmed(bool),
}
