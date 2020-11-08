use std::{error::Error, time::Duration, io};
use std::time::Instant;
use lsm9ds1::LSM9DS1;
use autopilot::{InputController, Input, ImuData, G};

use nalgebra::{Vector3};

use ahrs::{Ahrs};
use dsp::{Biquad, AlphaBetaGamma};
use dsp::biquad::low_pass;

pub struct LSM9DS1InputController<AHRS: Ahrs<f64>> {
	ahrs: AHRS,
	acc_offset: Vector3<f64>,
	acc_low_pass_filter: Biquad<f64>,
	acc_abg_filter: AlphaBetaGamma<f64>,
	gyr_offset: Vector3<f64>,
	gyr_low_pass_filter: Biquad<f64>,
	gyr_abg_filter: AlphaBetaGamma<f64>,
	last_data_instant: Option<Instant>,
	lsm9ds1: LSM9DS1,
}


impl<AHRS: Ahrs<f64>> LSM9DS1InputController<AHRS> {
	pub fn new(acc_gyr_path: &str,
			   mag_path: &str,
			   ahrs: AHRS) -> anyhow::Result<Self> {
		/*
		# gyro: lp: 170 hz, q = 0.45
		#       abg: (0.06, 0.004, 0.011)
		# acc:  lp: 145 hz, q = 0.48
		#       abg: (0.008, 0.0002, 0.)
		 */

		Ok(Self {
			lsm9ds1: LSM9DS1::new(acc_gyr_path, mag_path)?.init()?,
			ahrs,
			acc_offset: Vector3::<f64>::zeros(),
			gyr_offset: Vector3::<f64>::zeros(),
			acc_low_pass_filter: low_pass(145.0, 0.48, 500.0),
			acc_abg_filter: AlphaBetaGamma::<f64>::new(0.008, 0.0002, 0.0),
			gyr_low_pass_filter: low_pass(170.0, 0.45, 500.0),
			gyr_abg_filter: AlphaBetaGamma::<f64>::new(0.06, 0.004, 0.011),
			last_data_instant: None,
		})
	}

	pub fn calibrate(&mut self) -> Result<(Vector3<f64>, Vector3<f64>), io::Error> {
		//TODO: reject 20% extreme values

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


impl<AHRS: Ahrs<f64>> InputController for LSM9DS1InputController<AHRS> {
	const DELAY: Option<Duration> = Some(Duration::from_micros(1705)); // About 500 Hz

	fn read_input(&mut self) -> Result<Input, Box<dyn Error>> { // Function call is about 300 µs long
		let ([acc, gyr, mag], instant) = self.lsm9ds1.read_output()?;

		let processed_imu_data = {

			// Removing calibration offsets
			let acc = acc - &self.acc_offset;
			let gyr = gyr - &self.gyr_offset;

			let calibrated_acc = acc.clone();
			let calibrated_gyr = gyr.clone();

			// Low pass filter
			let acc = self.acc_low_pass_filter.update(acc);
			let gyr = self.gyr_low_pass_filter.update(gyr);

			let low_pass_filtered_acc: Vector3<f64> = acc.clone();
			let low_pass_filtered_gyr: Vector3<f64> = gyr.clone();

			if let Some(last_data_instant) = self.last_data_instant {

				// Alpha-beta-gamma filter
				let dt = (instant - last_data_instant).as_secs_f64();
				let acc = self.acc_abg_filter.update(acc, dt);
				let gyr = self.gyr_abg_filter.update(gyr, dt);

				let abg_filtered_acc: Vector3<f64> = acc.clone();
				let abg_filtered_gyr: Vector3<f64> = gyr.clone();

				// Updating AHRS
				if let Err(e) = self.ahrs.update_imu(&gyr, &acc, dt) {
					return Err(anyhow!("{:?}", e).into());
				}

				debug!(target: "lsm9ds1", "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
					   calibrated_acc[0], calibrated_acc[1], calibrated_acc[2],
					   calibrated_gyr[0], calibrated_gyr[1], calibrated_gyr[2],
					   low_pass_filtered_acc.x, low_pass_filtered_acc.y, low_pass_filtered_acc.z,
					   low_pass_filtered_gyr.x, low_pass_filtered_gyr.y, low_pass_filtered_gyr.z,
					   abg_filtered_acc.x, abg_filtered_acc.y, abg_filtered_acc.z,
					   abg_filtered_gyr.x, abg_filtered_gyr.y, abg_filtered_gyr.z,
				);

				ImuData {
					acc: abg_filtered_acc,
					gyr: abg_filtered_gyr,
					mag,
					instant,
				}
			} else {
				ImuData {
					acc: low_pass_filtered_acc,
					gyr: low_pass_filtered_gyr,
					mag,
					instant,
				}
			}
		};

		self.last_data_instant = Some(instant);

		let orientation = self.ahrs.quaternion();

		let as_euler_angles = orientation.euler_angles();
		debug!(target: "ahrs", "{} {} {}", as_euler_angles.0, as_euler_angles.1, as_euler_angles.2);

		Ok(Input::Orientation((orientation, processed_imu_data, instant)))
	}
}

