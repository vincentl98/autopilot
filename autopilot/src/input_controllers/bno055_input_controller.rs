use crate::input::{InputController, Input, ImuCalibrationStatus};
use crossbeam_channel::Sender;
use std::time::Duration;
use std::{thread};
use bno055::{Bno055, BNO055OperationMode, BNO055PowerMode, BNO055_ID, BNO055Calibration};
use rppal::i2c::I2c;

pub struct Bno055InputController {
	bno055: Bno055<I2c>
}

impl Bno055InputController {
	pub fn new(i2c: I2c, calibration_profile: Option<BNO055Calibration>) -> anyhow::Result<Self> {
		let mut instance = Self {
			bno055: Bno055::<I2c>::new(i2c)
		}
			.init(BNO055OperationMode::NDOF, calibration_profile)
			.map_err(|e| anyhow!("Failed to initialize: {:?}", e))?;

		instance.bno055
			.set_mode(BNO055OperationMode::NDOF, &mut rppal::hal::Delay {})
			.map_err(|e| anyhow!("Failed to set mode: {:?}", e))?;

		Ok(instance)
	}

	pub fn init(mut self, operation_mode: BNO055OperationMode, calibration_profile: Option<BNO055Calibration>) -> anyhow::Result<Self> {
		// The `init` function of the BNO055 is not used because its implementation is erroneous
		// This function also waits for the sensor to be at least partly calibrated

		let id = self.bno055.id().map_err(|e| anyhow!("Failed to read chip id: {:?}", e))?;
		if id != BNO055_ID {
			Err(anyhow!("Failed to identify BNO055: got {}, expected {}", id, BNO055_ID))
		} else {
			self.bno055.soft_reset().map_err(|e| anyhow!("Failed to soft reset: {:?}", e))?;
			thread::sleep(Duration::from_millis(650)); // cf datasheet

			self.bno055.set_power_mode(BNO055PowerMode::NORMAL).map_err(|e| anyhow!("Failed to set power mode: {:?}", e))?;

			if let Some(calibration_profile) = calibration_profile {
				self.bno055
					.set_mode(BNO055OperationMode::CONFIG_MODE, &mut rppal::hal::Delay {})
					.map_err(|e| anyhow!("Failed to change sensor mode: {:?}", e))?;

				self.bno055
					.set_calibration_profile(calibration_profile, &mut rppal::hal::Delay {})
					.map_err(|e| anyhow!("Failed to set calibration profile: {:?}", e))?;
			}

			self.bno055
				.set_mode(operation_mode, &mut rppal::hal::Delay {})
				.map_err(|e| anyhow!("Failed to change sensor mode: {:?}", e))?;

			thread::sleep(Duration::from_millis(400));


			Ok(self)
		}
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
					let system_calibration_status = calibration_status.sys;

					let imu_calibration_status = match system_calibration_status {
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
