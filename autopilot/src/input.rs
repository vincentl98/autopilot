use nalgebra::Vector3;
use std::time::{Instant};

pub const G: f32 = 9.80665;

#[derive(Clone, Debug)]
pub struct ImuData {
	pub acc: Vector3<f32>,
	pub gyr: Vector3<f32>,
	pub mag: Vector3<f32>,
	pub instant: Instant,
}

impl Default for ImuData {
	fn default() -> Self {
		ImuData {
			acc: Vector3::default(),
			gyr: Vector3::default(),
			mag: Vector3::default(),
			instant: Instant::now(),
		}
	}
}

#[derive(Clone, Debug, Default)]
pub struct NavioAdcData {
	pub board_voltage: f32,
	pub servo_voltage: f32,
	pub external_voltage: f32,
	pub external_current: f32,
	pub adc_port_2: f32,
	pub adc_port_3: f32,
}

pub type RcChannels = Option<[f32; 16]>;

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub enum Input {
	RcChannels(RcChannels),
	NavioAdc(NavioAdcData),
	Imu(ImuData),
	SoftArmed(bool),
}
