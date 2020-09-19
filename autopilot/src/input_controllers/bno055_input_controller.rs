use crate::input::{InputController, Input, ImuCalibrationStatus};
use crossbeam_channel::Sender;
use std::time::Duration;
use std::{thread};
use bno055::{Bno055, OperationMode, PowerMode, CalibrationProfile};
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
				}
				Err(e) => error!("Failed to read calibration status: {:?}", e)
			}

			// Data output rate is 100 Hz ie every 10 ms, cf datasheet
			const DATA_OUTPUT_RATE_DELAY: Duration = Duration::from_millis(10);
			thread::sleep(DATA_OUTPUT_RATE_DELAY);
		}
	}
}
