use ahrs::Madgwick;
use pid::Pid;
use crate::quadcopter::{QuadcopterInputFrame, QuadcopterOutputFrame, LedColor};
use std::time::{Duration, Instant};
use autopilot::Autopilot;
use crate::output_controllers::navio_esc_output_controller::QUADCOPTER_ESC_CHANNELS;
use std::f64::consts::PI;
use num_traits::Pow;


#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
	Armed,
	Disarmed,
	Off,
}

pub struct QuadcopterAutopilot {
	pid_roll: Pid<f64>,
	pid_pitch: Pid<f64>,
}

impl QuadcopterAutopilot {
	pub fn new(pid_roll: (f64, f64, f64), pid_pitch: (f64, f64, f64), _pid_yaw: (f64, f64, f64)) -> Self {
		Self {
			pid_roll: Pid::new(pid_roll, 0., Some((-1., 1.))),
			pid_pitch: Pid::new(pid_pitch, 0., Some((-1., 1.))),
		}
	}

	fn mode(&self, input_frame: &QuadcopterInputFrame) -> Mode {
		const MINIMAL_EXTERNAL_VOLTAGE: f64 = 10.0;
		// Arming conditions:
		// - Soft arm: true
		// - RC channels: Some
		// - RC channel n°8 (ie 7): > 0.5
		// - Battery voltage >= min
		if input_frame.soft_armed && input_frame.rc_channels.is_some() {
			if input_frame.rc_channels.unwrap()[4] > 0.5
				&& input_frame.navio_adc.external_voltage >= MINIMAL_EXTERNAL_VOLTAGE {
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

		let (orientation, instant) = input_frame.orientation;

		let (roll, pitch, _yaw) = orientation.euler_angles();

		// dbg!(roll * 180. / PI);
		// dbg!(roll_estimate * 180.);

		// dbg!(pitch * 180. / PI);
		// dbg!(pitch_estimate * 180.);

		// info!("{}", pitch * 180. / PI);

		match mode {
			// _ => {
			Mode::Armed => {
				let rc_channels = input_frame.rc_channels.unwrap();

				let roll_estimate = self.pid_roll.estimate(roll, instant);
				let pitch_estimate = self.pid_pitch.estimate(pitch, instant);

				let mut esc_channels = [0f64; QUADCOPTER_ESC_CHANNELS]; // between -2 and 2
				esc_channels[0] = pitch_estimate - roll_estimate;
				esc_channels[1] = -pitch_estimate - roll_estimate;
				esc_channels[2] = -pitch_estimate + roll_estimate;
				esc_channels[3] = pitch_estimate + roll_estimate;

				const MIN_ESC_VALUE: f64 = 0.08;

				for motor in esc_channels.iter_mut() {
					*motor = (*motor / 2. + rc_channels[2]).max(MIN_ESC_VALUE).min(1.);
				}

				// if rc_channels[2] != 0. {
				// info!("roll: {:.2} ({:.2}), pitch: {:.2} ({:.2})", //, esc: {:.2} {:.2} {:.2} {:.2}",
				// 	  roll * 180. / PI,
				// 	  roll_estimate * 180. / PI,
				// 	  pitch * 180. / PI,
				// 	  pitch_estimate * 180. / PI);
				// esc_channels[0],
				// esc_channels[1],
				// esc_channels[2],
				// esc_channels[3]);

				// dbg!(esc_channels);
				// }

				if rc_channels[5] > 0.5 {
					for motor in esc_channels.iter_mut() {
						*motor = rc_channels[2];
					}
				}


				// dbg!(esc_channels);

				// esc_channels[0] = rc_channels[2];
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
					// esc_channels:[0f64; QUADCOPTER_ESC_CHANNELS],
				}
			}
			Mode::Off => {
				QuadcopterOutputFrame {
					led: None,
					esc_channels: [0.; QUADCOPTER_ESC_CHANNELS],
				}
			}
			Mode::Disarmed => {
				self.pid_pitch.reset();
				self.pid_roll.reset();
				QuadcopterOutputFrame {
					led: Some(LedColor::Red),
					esc_channels: [0.; QUADCOPTER_ESC_CHANNELS],
				}
			}
		}
	}
}