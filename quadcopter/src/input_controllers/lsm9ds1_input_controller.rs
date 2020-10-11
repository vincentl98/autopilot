use crossbeam_channel::{unbounded, Receiver, Sender};

use std::{error::Error, time::Duration};
use spidev::{Spidev, SpidevOptions, SpiModeFlags, SpidevTransfer};
use std::io::{Write, Read};
use std::time::Instant;
use lsm9ds1::LSM9DS1;
use autopilot::{InputController, Input};

const NAVIO2_ACC_GYR_SPI_PATH: &'static str = "/dev/spidev0.3";
const NAVIO2_MAG_SPI_PATH: &'static str = "/dev/spidev0.2";

pub struct LSM9DS1InputController {
	lsm9ds1: LSM9DS1,
}

impl LSM9DS1InputController {
	pub fn new() -> anyhow::Result<Self> {
		Ok(Self {
			lsm9ds1: LSM9DS1::new(NAVIO2_ACC_GYR_SPI_PATH,
								  NAVIO2_MAG_SPI_PATH,
			)?.init()?
		})
	}
}

/*
pub fn calibrate(mut self) -> Result<Self, io::Error> {
		const CALIBRATION_MEASUREMENTS: usize = 100;
		let mut acc = Vector3::<f32>::new();
		let mut gyr = Vector3::<f32>::new();

		for _ in 0..CALIBRATION_MEASUREMENTS {
			let (imu_output, _) = self.read_output()?;
			acc = acc + imu_output.acc;
			gyr = gyr + imu_output.gyr;
			thread::sleep(OUTPUT_DELAY);
		}

		const INV_CALIBRATION_MEASUREMENTS: f32 = 1f32 / CALIBRATION_MEASUREMENTS as f32;
		acc = acc * INV_CALIBRATION_MEASUREMENTS;
		gyr = gyr * INV_CALIBRATION_MEASUREMENTS;

		acc.z -= G;

		self.acc_offset = acc;
		self.gyr_offset = gyr;

		Ok(self)
	}
 */
impl InputController for LSM9DS1InputController {
	const DELAY: Option<Duration> = Some(Duration::from_millis(10));

	fn read_input(&mut self) -> Result<Input, Box<dyn Error>> {
		let output = self.lsm9ds1.read_output()?;
		Ok(Input::Imu(output))
	}
}

