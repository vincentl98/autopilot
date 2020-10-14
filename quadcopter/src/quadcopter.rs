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

#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
	Armed,
	Disarmed,
	Off,
}

pub struct QuadcopterAutopilot {
	ahrs: Madgwick<f32>,
	roll_pid: Pid<f32>,
	pitch_pid: Pid<f32>,
}

impl QuadcopterAutopilot {
	pub fn new(beta: f32) -> Self {
		Self {
			ahrs: Madgwick::new(beta),
			roll_pid: Pid::new((0.05, 0., 0.), 0., Some((-1., 1.))),
			pitch_pid: Pid::new((0.05, 0., 0.), 0., Some((-1., 1.))),
		}
	}

	fn mode(&self, input_frame: &QuadcopterInputFrame) -> Mode {
		// Arming conditions:
		// - Soft arm: true
		// - RC channels: Some
		// - RC channel n°8 (ie 7): > 0.5
		// (maybe check battery voltage)
		if input_frame.soft_armed && input_frame.rc_channels.is_some() {
			if input_frame.rc_channels.unwrap()[7] > 0.5 {
				Mode::Armed
			} else {
				Mode::Disarmed
			}
		} else {
			Mode::Off
		}
	}
}

impl Autopilot<QuadcopterInputFrame, QuadcopterOutputFrame> for QuadcopterAutopilot {
	const MAX_CONTROL_LOOP_PERIOD: Duration = Duration::from_millis(50);

	fn output_frame(&mut self, input_frame: QuadcopterInputFrame) -> QuadcopterOutputFrame {
		let mode = self.mode(&input_frame);

		match mode {
			Mode::Off => {
				QuadcopterOutputFrame {
					led: None,
					esc_channels: [0f32; QUADCOPTER_ESC_CHANNELS],
				}
			}
			Mode::Disarmed => {
				QuadcopterOutputFrame {
					led: Some(LedColor::Red),
					esc_channels: [0f32; QUADCOPTER_ESC_CHANNELS],
				}
			}
			Mode::Armed => {
				let rc_channels = input_frame.rc_channels.unwrap();
				let mut esc_channels = [0f32; QUADCOPTER_ESC_CHANNELS];

				let (roll, pitch, yaw) = self.ahrs
					.update_imu(
						input_frame.imu.gyr,
						input_frame.imu.acc,
						input_frame.imu.instant)
					.euler_angles();

				let roll_estimate = self.roll_pid.estimate(roll, input_frame.imu.instant);
				let pitch_estimate = self.pitch_pid.estimate(pitch, input_frame.imu.instant);
				// dbg!(roll * 180.);
				// dbg!(roll_estimate);

				// dbg!(pitch * 180.);
				dbg!(pitch_estimate);
				esc_channels[0] = rc_channels[2];
				// if let Ok(q) = self.ahrs.update_imu(
				// 	input_frame.imu.gyr,
				// 	input_frame.imu.acc,
				// 	input_frame.imu.dt) {
				// 	dbg!(q.euler_angles());
				// } else {
				// 	eprintln!("Failed to compute imu");
				// }

				QuadcopterOutputFrame {
					led: Some(LedColor::Green),
					esc_channels,
				}
			}
		}
	}
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
