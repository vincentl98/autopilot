use crossbeam_channel::{unbounded, Receiver, Sender};

use std::{error::Error, time::Duration, io, thread};
use spidev::{Spidev, SpidevOptions, SpiModeFlags, SpidevTransfer};
use std::io::{Write, Read};
use std::time::Instant;
use lsm9ds1::LSM9DS1;
use autopilot::{InputController, Input, ImuData, G};

use nalgebra::Vector3;
use num_traits::identities::Zero;


pub struct LSM9DS1InputController {
	lsm9ds1: LSM9DS1,
	acc_offset: Vector3<f32>,
	gyr_offset: Vector3<f32>,
}

impl LSM9DS1InputController {
	pub fn new(acc_gyr_path: &str, mag_path: &str) -> anyhow::Result<Self> {
		Ok(Self {
			lsm9ds1: LSM9DS1::new(acc_gyr_path, mag_path)?.init()?,
			acc_offset: Vector3::<f32>::zeros(),
			gyr_offset: Vector3::<f32>::zeros(),
		})
	}

	pub fn calibrate(&mut self) -> Result<(), io::Error> {
		const CALIBRATION_MEASUREMENTS: usize = 500;
		let mut acc_average = Vector3::<f32>::zeros();
		let mut gyr_average = Vector3::<f32>::zeros();

		info!("Begin LSM9DS1 calibration");
		for _ in 0..CALIBRATION_MEASUREMENTS {
			let ([acc, gyr, _], _) = self.lsm9ds1.read_output()?;
			acc_average = acc_average + acc;
			gyr_average = gyr_average + gyr;
			std::thread::sleep(lsm9ds1::OUTPUT_DELAY);
		}

		acc_average = acc_average / CALIBRATION_MEASUREMENTS as f32;
		gyr_average = gyr_average / CALIBRATION_MEASUREMENTS as f32;

		acc_average.z -= G;

		self.acc_offset = acc_average;
		self.gyr_offset = gyr_average;

		info!("Acc offset: {}", &self.acc_offset);
		info!("Gyr offset: {}", &self.gyr_offset);

		Ok(())
	}
}


impl InputController for LSM9DS1InputController {
	const DELAY: Option<Duration> = Some(Duration::from_millis(10));

	fn read_input(&mut self) -> Result<Input, Box<dyn Error>> {
		let ([acc, gyr, mag], instant) = self.lsm9ds1.read_output()?;
		let corrected_output = ImuData {
			gyr: gyr - &self.gyr_offset,
			acc: acc - &self.acc_offset,
			mag,
			instant,
		};

		Ok(Input::Imu(corrected_output))
	}
}

