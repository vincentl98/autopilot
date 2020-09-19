use crate::input::{InputController, Input, ImuCalibrationStatus};
use crossbeam_channel::Sender;
use std::time::{Duration, Instant};
use std::{thread};
use bno055::{Bno055, OperationMode, PowerMode, CalibrationProfile, CalibrationStatus};
use rppal::i2c::I2c;
use rppal::hal::Delay;

pub struct Bno055InputController {
	bno055: Bno055<I2c>
}

impl Bno055InputController {
	pub fn new(i2c: I2c, calibration_profile: Option<CalibrationProfile>) -> anyhow::Result<Self> {
		let mut instance = Self {
			bno055: Bno055::<I2c>::new(i2c)
		};

		instance.bno055
			.init(OperationMode::NDOF, calibration_profile, &mut Delay {})
			.map_err(|e| anyhow!("Failed to initialize: {:?}", e))?;

		Ok(instance)
	}
}

impl InputController for Bno055InputController {
	fn read_input(&mut self, input_sender: Sender<Input>) -> ! {
		// Calibration status should be monitored, but it is not necessary to read it on every
		// iteration. An "async delay" pattern is used, as `Bno055` objects cannot be shared between
		// threads (need a &mut).

		let mut last_calibration_status_instant = Instant::now();
		const CALIBRATION_STATUS_READ_DELAY: Duration = Duration::from_secs(1);

		loop {
			match self.bno055.euler_angles() {
				Ok(euler_angles) => {
					// println!("euler angles: {:?}", &euler_angles);
					input_sender
						.send(Input::OrientationEuler(euler_angles))
						.map_err(|e| error!("{}", e))
						.unwrap_or_default()
				}
				Err(e) => error!("Failed to read sensor data: {:?}", e)
			}

			if Instant::now() - last_calibration_status_instant >= CALIBRATION_STATUS_READ_DELAY {
				match self.bno055.get_calibration_status() {
					Ok(calibration_status) => {
						let imu_calibration_status = match calibration_status.all {
							0 => ImuCalibrationStatus::Lowest,
							1 => ImuCalibrationStatus::Low,
							2 => ImuCalibrationStatus::High,
							3 => ImuCalibrationStatus::Highest,
							_ => ImuCalibrationStatus::Unknown,
						};

						input_sender
							.send(Input::ImuCalibrationStatus(imu_calibration_status))
							.map_err(|e| error!("{}", e))
							.unwrap_or_default();

						last_calibration_status_instant = Instant::now();
					}
					Err(e) => error!("Failed to read calibration status: {:?}", e)
				}
			}


			// Data output rate is 100 Hz ie every 10 ms, cf datasheet
			const DATA_OUTPUT_RATE_DELAY: Duration = Duration::from_millis(10);
			thread::sleep(DATA_OUTPUT_RATE_DELAY);
		}
	}
}
