use crossbeam_channel::Sender;
use std::time::Instant;

use autopilot::{Collector, Input, Dispatcher, ImuData, RcChannels, NavioAdcData, Orientation};
use nalgebra::{UnitQuaternion, Quaternion};

use crate::output_controllers::navio_esc_output_controller::QUADCOPTER_ESC_CHANNELS;

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
	pub esc_channels: [f64; QUADCOPTER_ESC_CHANNELS],
}

pub struct QuadcopterDispatcher {
	pub led_sender: Sender<Option<LedColor>>,
	pub esc_channels_sender: Sender<[f64; QUADCOPTER_ESC_CHANNELS]>,
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
			input_frame: QuadcopterInputFrame {
				navio_adc: NavioAdcData::default(),
				orientation: (UnitQuaternion::from_quaternion(Quaternion::new(1., 0., 0., 0.)),
							  ImuData::default(),
							  Instant::now()),
				rc_channels: RcChannels::default(),
				soft_armed: false,
			}
		}
	}
}

#[derive(Debug, Clone)]
pub struct QuadcopterInputFrame {
	pub navio_adc: NavioAdcData<f64>,
	pub orientation: Orientation<f64>,
	pub rc_channels: RcChannels<f64>,
	pub soft_armed: bool,
}

impl Collector<QuadcopterInputFrame> for QuadcopterCollector {
	fn collect(&mut self, input: Input) -> QuadcopterInputFrame {
		#[allow(unreachable_patterns)]
		match input {
			Input::Orientation(orientation) => self.input_frame.orientation = orientation,
			Input::NavioAdc(navio_adc) => self.input_frame.navio_adc = navio_adc,
			Input::RcChannels(rc_channels) => self.input_frame.rc_channels = rc_channels,
			Input::SoftArmed(soft_armed) => self.input_frame.soft_armed = soft_armed,
			_ => error!("Unhandled input: {:?}", input),
		}

		self.input_frame.clone()
	}
}
