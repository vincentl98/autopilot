use ahrs::Madgwick;
use pid::Pid;
use crate::quadcopter::{QuadcopterInputFrame, QuadcopterOutputFrame, LedColor};
use std::time::Duration;
use autopilot::Autopilot;
use crate::output_controllers::navio_esc_output_controller::QUADCOPTER_ESC_CHANNELS;


#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
	Armed,
	Disarmed,
	Off,
}

pub struct QuadcopterAutopilot {
	ahrs: Madgwick<f32>,
	pid_roll: Pid<f32>,
	pid_pitch: Pid<f32>,
}

impl QuadcopterAutopilot {
	pub fn new(beta: f32, pid_roll: (f32, f32, f32), pid_pitch: (f32, f32, f32), pid_yaw: (f32, f32, f32)) -> Self {
		Self {
			ahrs: Madgwick::new(beta),
			pid_roll: Pid::new(pid_roll, 0., Some((-1., 1.))),
			pid_pitch: Pid::new(pid_pitch, 0., Some((-1., 1.))),
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

				let roll_estimate = self.pid_roll.estimate(roll, input_frame.imu.instant);
				let pitch_estimate = self.pid_pitch.estimate(pitch, input_frame.imu.instant);
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