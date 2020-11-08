use log::LevelFilter;
use std::error::Error;
use std::fs::{File, OpenOptions};
use std::f64::consts::PI;
use std::io::Write;
use serde::{Serialize, Deserialize};

use crate::roll_pitch_yaw::{RollPitchYaw, RollPitch};

#[derive(Serialize, Deserialize)]
pub struct QuadcopterConfig {
	pub log_level_filter: String,
	pub pid_values: RollPitchYaw<(f64, f64, f64)>,
	pub rates: RollPitchYaw<f64>,
	pub limits: RollPitch<f64>,
	pub calibration_acc: [f64; 3],
	pub calibration_gyr: [f64;3],
	pub ahrs_madgwick_beta: f64,
	pub input_rc_range: (u16, u16),
	pub output_esc_pins: [u32; 4],
	pub filter_gyr_abg: (f64, f64, f64),
	pub filter_gyr_low_pass: (f64, f64, f64),
	pub filter_acc_abg: (f64, f64, f64),
	pub filter_acc_low_pass: (f64, f64, f64),
	pub filter_d_term_abg: (f64, f64, f64),
}

pub trait TryIntoLevelFilter {
	fn try_into_level_filter(&self) -> Result<LevelFilter, ()>;
}

impl TryIntoLevelFilter for String {
	fn try_into_level_filter(&self) -> Result<LevelFilter, ()> {
		Ok(match self.as_str() {
			"none" => LevelFilter::Off,
			"error" => LevelFilter::Error,
			"warn" => LevelFilter::Warn,
			"info" => LevelFilter::Info,
			"debug" => LevelFilter::Debug,
			"all" => LevelFilter::Trace,
			_ => return Err(()),
		})
	}
}

impl Default for QuadcopterConfig {
	fn default() -> Self {
		QuadcopterConfig {
			log_level_filter: String::from("all"),
			pid_values: RollPitchYaw {
				roll: (0.135, 0., 0.),
				pitch: (0.135, 0., 0.),
				yaw: (0.135,0.,0.),
			},
			rates: RollPitchYaw {
				roll: 2. * PI,
				pitch: 2. * PI,
				yaw: 4. * PI,
			},
			limits: RollPitch {
				roll: PI / 4.,
				pitch: PI / 4.,
			},
			calibration_acc: [0., 0., 0.],
			calibration_gyr: [0., 0., 0.],
			ahrs_madgwick_beta: 1.15,
			input_rc_range: (1000, 2000),
			output_esc_pins: [0, 1, 2, 3],
			filter_gyr_abg: (0.0, 0.0, 0.0),
			filter_gyr_low_pass: (0.0, 0.0, 0.0),
			filter_acc_abg: (0.0, 0.0, 0.0),
			filter_acc_low_pass: (0.0, 0.0, 0.0),
			filter_d_term_abg: (0.0, 0.0, 0.0)
		}
	}
}

const CONFIG_FILE_PATH: &'static str = "config.ini";

pub fn read() -> Result<QuadcopterConfig, Box<dyn Error>> {
	let config_file = File::open(CONFIG_FILE_PATH)?;

	let config: QuadcopterConfig = serde_json::from_reader(config_file)?;

	Ok(config)
}

pub fn save(config: &QuadcopterConfig) -> Result<(), Box<dyn Error>> {

	let mut config_file = OpenOptions::new()
		.create(true)
		.write(true)
		.truncate(true)
		.open(CONFIG_FILE_PATH)?;

	write!(config_file, "{}", serde_json::to_string(config)?)?;

	Ok(())
}
