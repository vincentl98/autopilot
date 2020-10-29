use crossbeam_channel::{unbounded, Receiver, Sender};

use std::{error::Error, time::Duration, io, thread};
use spidev::{Spidev, SpidevOptions, SpiModeFlags, SpidevTransfer};
use std::io::{Write, Read};
use std::time::Instant;
use lsm9ds1::LSM9DS1;
use autopilot::{InputController, Input, ImuData, G};

use nalgebra::{Vector3, RealField};
use num_traits::identities::Zero;

use ahrs::{Mahony};
use dsp::AlphaBetaFilter;
use std::f64::consts::PI;

pub struct LSM9DS1InputController<AHRS: Ahrs<f64>> {
	ahrs: AHRS<f64>,
	acc_offset: Vector3<f64>,
	acc_filter: AlphaBetaFilter<f64>,
	gyr_offset: Vector3<f64>,
	gyr_filter: AlphaBetaFilter<f64>,
	last_data_instant: Option<Instant>,
	lsm9ds1: LSM9DS1,
}

impl<AHRS: Ahrs<f64>> LSM9DS1InputController<AHRS> {
	pub fn new(acc_gyr_path: &str,
			   mag_path: &str,
			   acc_filter_alpha_beta: (f64, f64),
			   gyr_filter_alpha_beta: (f64, f64),
			   ahrs: AHRS<f64>) -> anyhow::Result<Self> {
		Ok(Self {
			ahrs,
			acc_offset: Vector3::<f64>::zeros(),
			acc_filter: AlphaBetaFilter::new(acc_filter_alpha_beta.0, acc_filter_alpha_beta.1),
			gyr_offset: Vector3::<f64>::zeros(),
			gyr_filter: AlphaBetaFilter::new(gyr_filter_alpha_beta.0, gyr_filter_alpha_beta.1),
			last_data_instant: None,
			lsm9ds1: LSM9DS1::new(acc_gyr_path, mag_path)?.init()?,
		})
	}

	pub fn calibrate(&mut self) -> Result<(Vector3<f64>, Vector3<f64>), io::Error> {
		const CALIBRATION_MEASUREMENTS: usize = 500;

		let mut acc_average = Vector3::<f64>::zeros();
		let mut gyr_average = Vector3::<f64>::zeros();

		for _ in 0..CALIBRATION_MEASUREMENTS {
			let ([acc, gyr, _], _) = self.lsm9ds1.read_output()?;
			acc_average = acc_average + acc;
			gyr_average = gyr_average + gyr;
			std::thread::sleep(lsm9ds1::OUTPUT_DELAY);
		}

		acc_average = acc_average / CALIBRATION_MEASUREMENTS as f64;
		gyr_average = gyr_average / CALIBRATION_MEASUREMENTS as f64;

		acc_average.z -= G;

		Ok((acc_average, gyr_average))
	}

	pub fn set_calibration(&mut self, acc_offset: Vector3<f64>, gyr_offset: Vector3<f64>) {
		self.acc_offset = acc_offset;
		self.gyr_offset = gyr_offset;
	}
}


impl<AHRS> InputController for LSM9DS1InputController<AHRS> {
	const DELAY: Option<Duration> = Some(Duration::from_micros(1700)); // About 500 Hz

	fn read_input(&mut self) -> Result<Input, Box<dyn Error>> { // Function call is about 300 µs long
		let ([acc, gyr, _mag], instant) = self.lsm9ds1.read_output()?;

		if let Some(last_data_instant) = self.last_data_instant {

			// Removing calibration offsets
			let acc = acc - &self.acc_offset;
			let gyr = gyr - &self.gyr_offset;

			let calibrated_acc = acc.clone();
			let calibrated_gyr = gyr.clone();

			// Filtering gyroscope data
			let dt = (instant - last_data_instant).as_secs_f64();
			let acc = self.acc_filter.update(acc, dt);
			let gyr = self.gyr_filter.update(gyr, dt);

			let filtered_acc = acc.clone();
			let filtered_gyr = gyr.clone();

			// Updating AHRS

			if let Ok(orientation) = self.ahrs.update_imu(&gyr, &acc, dt)  {
				trace!("LSM9DS1: {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
					   calibrated_acc[0], calibrated_acc[1], calibrated_acc[2],
					   calibrated_gyr[0], calibrated_gyr[1], calibrated_gyr[2],
					   filtered_acc[0], filtered_acc[1], filtered_acc[2],
					   filtered_gyr[0], filtered_gyr[1], filtered_gyr[2],
					   orientation.0 * 180. / PI,
					   orientation.1 * 180. / PI,
					   orientation.2 * 180. / PI);
			} else {
				return Err(anyhow!("{:?}", e).into());
			}

		}

		self.last_data_instant = Some(instant);


		Ok(Input::Orientation((self.ahrs.last_quaternion(), instant)))
	}
}

