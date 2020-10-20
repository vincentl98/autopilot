use crossbeam_channel::Sender;
use std::time::Instant;

use ahrs::{Madgwick};
use systemstat::Duration;
use autopilot::{Autopilot, Collector, Input, Dispatcher, ImuData, RcChannels, NavioAdcData};
use std::f32::consts::PI;
use std::ops::Mul;
use crate::output_controllers::navio_esc_output_controller::QUADCOPTER_ESC_CHANNELS;
use pid::Pid;

#[allow(dead_code)]
#[derive(Debug)]
pub enum LedColor {
	Red,
	Green,
	Blue,
	Cyan,
	Magenta,
	Yellow,
	White,
}

#[derive(Debug)]
pub struct QuadcopterOutputFrame {
	pub led: Option<LedColor>,
	pub esc_channels: [f32; QUADCOPTER_ESC_CHANNELS],
}

pub struct QuadcopterDispatcher {
	pub led_sender: Sender<Option<LedColor>>,
	pub esc_channels_sender: Sender<[f32; QUADCOPTER_ESC_CHANNELS]>,
}

impl Dispatcher<QuadcopterOutputFrame> for QuadcopterDispatcher {
	fn dispatch(&self, output_frame: QuadcopterOutputFrame) {
		self.led_sender.send(output_frame.led).unwrap();
		self.esc_channels_sender.send(output_frame.esc_channels).unwrap();
	}
}

pub struct QuadcopterCollector {
	input_frame: QuadcopterInputFrame
}

impl QuadcopterCollector {
	pub fn new() -> Self {
		Self {
			input_frame: QuadcopterInputFrame::default()
		}
	}
}

#[derive(Debug, Clone, Default)]
pub struct QuadcopterInputFrame {
	pub imu: ImuData,
	pub navio_adc: NavioAdcData,
	pub rc_channels: RcChannels,
	pub soft_armed: bool,
}

impl Collector<QuadcopterInputFrame> for QuadcopterCollector {
	fn collect(&mut self, input: Input) -> QuadcopterInputFrame {
		match input {
			Input::Imu(imu_data) => self.input_frame.imu = imu_data,
			Input::NavioAdc(navio_adc) => self.input_frame.navio_adc = navio_adc,
			Input::RcChannels(rc_channels) => self.input_frame.rc_channels = rc_channels,
			Input::SoftArmed(soft_armed) => self.input_frame.soft_armed = soft_armed,
			_ => warn!("Unhandled input: {:?}", input),
		}

		self.input_frame.clone()
	}
}
