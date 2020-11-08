use std::time::{Duration};

use autopilot::Autopilot;
use pid::Pid;

use crate::quadcopter::{QuadcopterInputFrame, QuadcopterOutputFrame, LedColor};
use crate::output_controllers::navio_esc_output_controller::QUADCOPTER_ESC_CHANNELS;
use crate::roll_pitch_yaw::{RollPitchYaw, RollPitch};
use crate::mixer::Mixer;


#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
	Armed,
	Disarmed,
	Off,
}

pub struct QuadcopterAutopilot {
	pids: RollPitchYaw<Pid<f64>>,
	rates: RollPitchYaw<f64>,
	limits: RollPitch<f64>,
	mixer: Mixer<f64>,
	previous_mode: Mode,
}

impl QuadcopterAutopilot {
	pub fn new(pid_values: RollPitchYaw<(f64, f64, f64)>,
			   rates: RollPitchYaw<f64>,
			   limits: RollPitch<f64>) -> Self {
		const MIN_ESC_VALUE: f64 = 0.025;
		Self {
			pids: RollPitchYaw {
				roll: Pid::new(pid_values.roll, 0., Some((-1., 1.))),
				pitch: Pid::new(pid_values.pitch, 0., Some((-1., 1.))),
				yaw: Pid::new(pid_values.yaw, 0., Some((-1., 1.))),
			},
			rates,
			limits,
			mixer: Mixer { min_output: MIN_ESC_VALUE },
			previous_mode: Mode::Off,
		}
	}

	fn mode(&self, input_frame: &QuadcopterInputFrame) -> Mode {
		const MINIMAL_EXTERNAL_VOLTAGE: f64 = 10.0;
		//TODO: add max current

		// const MAXIMAL_EXTERNAL_CURRENT: f64 = 50.0;
		// Arming conditions:
		// - Soft arm: true
		// - RC channels: Some
		// - RC channel n°8 (ie 7): > 0.5
		// - Battery voltage >= min
		if input_frame.soft_armed && input_frame.rc_channels.is_some() {
			if input_frame.rc_channels.unwrap()[4] > 0.5 {
				if input_frame.navio_adc.external_voltage <= 2.0 {
					Mode::Armed
				} else if input_frame.navio_adc.external_voltage >= MINIMAL_EXTERNAL_VOLTAGE {
					Mode::Armed
				} else {
					match self.previous_mode {
						Mode::Armed => {
							warn!("Ext. voltage is low: {:.2} V", input_frame.navio_adc.external_voltage);
							Mode::Armed
						}
						_ => Mode::Disarmed
					}
				}
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
		self.previous_mode = mode;

		match mode {
			Mode::Armed => {
				let rc_channels = input_frame.rc_channels.unwrap();

				// Orientation

				// Target is set differently for yaw, as input controls angular rate on yaw axis.
				let target_orientation = RollPitchYaw {
					roll: (rc_channels[0] - 0.5) * self.limits.roll,
					pitch: (rc_channels[1] - 0.5) * self.limits.pitch,
					yaw: 0.0, // TODO: keep yaw constant, currently ignored
				};

				debug!(target: "target_orientation", "{} {} {}",
					   target_orientation.roll,
					   target_orientation.pitch,
					   target_orientation.yaw);

				let (quaternion,
					imu_data,
					instant) = input_frame.orientation;

				let current_orientation: RollPitchYaw<f64> = quaternion.euler_angles().into();

				debug!(target: "current_orientation", "{} {} {}",
					   current_orientation.roll,
					   current_orientation.pitch,
					   current_orientation.yaw);

				let delta_orientation = target_orientation - current_orientation;

				// Rates
				let target_rates = RollPitchYaw {
					roll: delta_orientation.roll * self.rates.roll,
					pitch: delta_orientation.pitch * self.rates.pitch,
					yaw: (rc_channels[3] - 0.5) * self.rates.yaw,
				};

				debug!(target: "target_rates", "{} {} {}",
					   target_rates.roll,
					   target_rates.pitch,
					   target_rates.yaw);

				self.pids.roll.set_setpoint(target_rates.roll);
				self.pids.pitch.set_setpoint(target_rates.pitch);
				self.pids.yaw.set_setpoint(target_rates.yaw);

				let pid_outputs = RollPitchYaw {
					roll: self.pids.roll.estimate(imu_data.gyr.x, instant),
					pitch: self.pids.pitch.estimate(imu_data.gyr.y, instant),
					yaw: self.pids.yaw.estimate(imu_data.gyr.z, instant),
				};

				info!(target: "pid_outputs", "{} {} {}", pid_outputs.roll, pid_outputs.pitch, pid_outputs.yaw);

				let mut outputs = self.mixer.mix(pid_outputs,
												 rc_channels[2]);

				if rc_channels[5] > 0.5 {
					for output in outputs.iter_mut() {
						*output = rc_channels[2];
					}
				}

				QuadcopterOutputFrame {
					led: Some(LedColor::Green),
					esc_channels: outputs,
				}
			}
			Mode::Off => {
				QuadcopterOutputFrame {
					led: None,
					esc_channels: [0.; QUADCOPTER_ESC_CHANNELS],
				}
			}
			Mode::Disarmed => {
				QuadcopterOutputFrame {
					led: Some(LedColor::Red),
					esc_channels: [0.; QUADCOPTER_ESC_CHANNELS],
				}
			}
		}
	}
}