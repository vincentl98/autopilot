use crossbeam_channel::Sender;
use std::time::Instant;

use systemstat::Duration;
use ahrs::MadgwickAhrs;
use autopilot::{Autopilot, Collector, Input, Dispatcher, ImuData};

#[derive(Debug, Clone, Default)]
pub struct QuadcopterInputFrame {
	pub soft_armed: Option<bool>,
	pub imu: Option<(ImuData, Instant)>,
}

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
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
	ArmedControlled,
	ArmedAutonomous,
	Disarmed,
	Failsafe,
}

pub struct QuadcopterAutopilot {
	ahrs: MadgwickAhrs,
	last_imu_instant: Option<Instant>,
}

impl QuadcopterAutopilot {
	pub fn new() -> Self {
		Self {
			ahrs: MadgwickAhrs::new(2f32),
			last_imu_instant: None,
		}
	}
}

impl Autopilot<QuadcopterInputFrame, QuadcopterOutputFrame> for QuadcopterAutopilot {
	const MAX_CONTROL_LOOP_PERIOD: Duration = Duration::from_millis(50);

	fn output_frame(&mut self, input_frame: QuadcopterInputFrame) -> QuadcopterOutputFrame {
		if let Some(imu) = input_frame.imu {
			if let Some(instant) = self.last_imu_instant {
				let dt = (imu.1 - instant).as_secs_f32();
				self.last_imu_instant = Some(imu.1);
				let q = self.ahrs.update(imu.0, dt);
				dbg!(q.euler_angles());
			} else {
				self.last_imu_instant = Some(imu.1);
			}
		}
		if let Some(soft_armed) = input_frame.soft_armed {
			if soft_armed {
				QuadcopterOutputFrame { led: Some(LedColor::Green) }
			} else {
				QuadcopterOutputFrame { led: None }
			}
		} else {
			QuadcopterOutputFrame { led: None }
		}
	}
}

pub struct QuadcopterDispatcher {
	pub led_sender: Sender<Option<LedColor>>,
}

impl Dispatcher<QuadcopterOutputFrame> for QuadcopterDispatcher {
	fn dispatch(&self, output_frame: QuadcopterOutputFrame) {
		self.led_sender.send(output_frame.led).unwrap();
	}
}

pub struct QuadcopterCollector {
	input_frame: QuadcopterInputFrame,
}

impl QuadcopterCollector {
	pub fn new() -> Self {
		Self {
			input_frame: QuadcopterInputFrame {
				soft_armed: None,
				imu: None,
			},
		}
	}
}

impl Collector<QuadcopterInputFrame> for QuadcopterCollector {
	fn collect(&mut self, input: Input) -> QuadcopterInputFrame {
		match input {
			Input::SoftArmed(armed) => self.input_frame.soft_armed = Some(armed),
			Input::Imu(data_instant) => self.input_frame.imu = Some(data_instant),
			_ => warn!("Unhandled input: {:?}", input),
		}
		self.input_frame.clone()
	}
}
