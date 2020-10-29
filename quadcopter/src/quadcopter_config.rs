use log::LevelFilter;
use tini::Ini;
use std::convert::TryFrom;
use nalgebra::Vector3;

pub struct QuadcopterConfig {
	pub log_level_filter: LevelFilter,
	pub pid_pitch: (f64, f64, f64),
	pub pid_roll: (f64, f64, f64),
	pub pid_yaw: (f64, f64, f64),
	pub offset_acc: Vector3<f64>,
	pub offset_gyr: Vector3<f64>,
	pub ahrs_madgwick_beta: f64,
	pub input_rc_range: (u16, u16),
	pub output_esc_pins: [u32; 4],
	pub filter_gyroscope: (f64, f64),
	pub filter_accelerometer: (f64, f64),
}

impl TryFrom<&Ini> for QuadcopterConfig {
	type Error = anyhow::Error;

	fn try_from(ini: &Ini) -> Result<Self, Self::Error> {
		const LOG_SECTION: &'static str = "log";
		const LEVEL_FILTER: &'static str = "level";

		let log_level_filter = match ini
			.get::<String>(LOG_SECTION, LEVEL_FILTER)
			.ok_or(Self::error(LOG_SECTION, LEVEL_FILTER))?
			.as_str()
		{
			"none" => LevelFilter::Off,
			"error" => LevelFilter::Error,
			"warn" => LevelFilter::Warn,
			"info" => LevelFilter::Info,
			"debug" => LevelFilter::Debug,
			"all" => LevelFilter::Trace,
			other => return Err(anyhow!("Invalid log level filter \"{}\"", other)),
		};

		const PID_YAW_SECTION: &'static str = "pid.yaw";
		const PID_PITCH_SECTION: &'static str = "pid.pitch";
		const PID_ROLL_SECTION: &'static str = "pid.roll";

		let pid_yaw = Self::read_pid(&ini, PID_YAW_SECTION)?;
		let pid_pitch = Self::read_pid(&ini, PID_PITCH_SECTION)?;
		let pid_roll = Self::read_pid(&ini, PID_ROLL_SECTION)?;

		const AHRS_MADGWICK_SECTION: &'static str = "ahrs.madgwick";
		const BETA: &'static str = "beta";

		let ahrs_madgwick_beta = ini
			.get::<f64>(AHRS_MADGWICK_SECTION, BETA)
			.ok_or(Self::error(AHRS_MADGWICK_SECTION, BETA))?;

		const RC_CHANNELS_RANGE_SECTION: &'static str = "input.rc.range";
		const MIN: &'static str = "min";
		const MAX: &'static str = "max";

		let rc_channels_range_min = ini
			.get::<u16>(RC_CHANNELS_RANGE_SECTION, MIN)
			.ok_or(Self::error(RC_CHANNELS_RANGE_SECTION, MIN))?;

		let rc_channels_range_max = ini
			.get::<u16>(RC_CHANNELS_RANGE_SECTION, MAX)
			.ok_or(Self::error(RC_CHANNELS_RANGE_SECTION, MAX))?;

		const OUTPUT_ESC_SECTION: &'static str = "output.esc";
		const ESC_COUNT: usize = 4;
		const PINS: &'static str = "pins";

		let output_esc_pins = ini
			.get_vec::<u32>(OUTPUT_ESC_SECTION, PINS)
			.ok_or(Self::error(OUTPUT_ESC_SECTION, PINS))?;

		if output_esc_pins.len() != ESC_COUNT {
			return Err(anyhow!("Invalid number of pins (expected {}, found {})", ESC_COUNT, output_esc_pins.len()));
		}

		const OFFSET_SECTION: &'static str = "offset";
		const OFFSET_LEN: usize = 3;
		const ACC: &'static str = "acc";

		let offset_acc = ini
			.get_vec::<f64>(OFFSET_SECTION, ACC)
			.ok_or(Self::error(OFFSET_SECTION, ACC))?;

		if offset_acc.len() != OFFSET_LEN {
			return Err(anyhow!("Invalid accelerometer offset length (expected {}, found {})",
			OFFSET_LEN, offset_acc.len()));
		}

		const GYR: &'static str = "gyr";

		let offset_gyr = ini
			.get_vec::<f64>(OFFSET_SECTION, GYR)
			.ok_or(Self::error(OFFSET_SECTION, GYR))?;

		if offset_gyr.len() != OFFSET_LEN {
			return Err(anyhow!("Invalid gyroscope offset length (expected {}, found {})",
			OFFSET_LEN, offset_gyr.len()));
		}

		const FILTER_GYROSCOPE_SECTION: &'static str = "filter.gyroscope";
		const ALPHA: &'static str = "alpha";

		let gyr_filter_alpha = ini
			.get::<f64>(FILTER_GYROSCOPE_SECTION, ALPHA)
			.ok_or(Self::error(FILTER_GYROSCOPE_SECTION, ALPHA))?;

		let gyr_filter_beta = ini
			.get::<f64>(FILTER_GYROSCOPE_SECTION, BETA)
			.ok_or(Self::error(FILTER_GYROSCOPE_SECTION, BETA))?;

		const FILTER_ACCELEROMETER_SECTION: &'static str = "filter.accelerometer";

		let acc_filter_alpha = ini
			.get::<f64>(FILTER_ACCELEROMETER_SECTION, ALPHA)
			.ok_or(Self::error(FILTER_ACCELEROMETER_SECTION, ALPHA))?;

		let acc_filter_beta = ini
			.get::<f64>(FILTER_ACCELEROMETER_SECTION, BETA)
			.ok_or(Self::error(FILTER_ACCELEROMETER_SECTION, BETA))?;


		Ok(Self {
			pid_roll,
			pid_yaw,
			pid_pitch,
			ahrs_madgwick_beta,
			log_level_filter,
			input_rc_range: (rc_channels_range_min, rc_channels_range_max),
			output_esc_pins: [output_esc_pins[0], output_esc_pins[1], output_esc_pins[2], output_esc_pins[3]],
			offset_acc: Vector3::new(offset_acc[0], offset_acc[1], offset_acc[2]),
			offset_gyr: Vector3::new(offset_gyr[0], offset_gyr[1], offset_gyr[2]),
			filter_gyroscope: (gyr_filter_alpha, gyr_filter_beta),
			filter_accelerometer: (acc_filter_alpha, acc_filter_beta),
		})
	}
}

impl QuadcopterConfig {
	fn read_pid(ini: &Ini, section: &str) -> Result<(f64, f64, f64), anyhow::Error> {
		const P: &'static str = "p";
		const I: &'static str = "i";
		const D: &'static str = "d";

		let p: f64 = ini
			.get::<f64>(section, P)
			.ok_or(Self::error(section, P))?;

		let i: f64 = ini
			.get::<f64>(section, I)
			.ok_or(Self::error(section, I))?;

		let d: f64 = ini
			.get::<f64>(section, D)
			.ok_or(Self::error(section, D))?;

		Ok((p, i, d))
	}

	fn error(section: &str, key: &str) -> anyhow::Error {
		anyhow!("Failed to read configuration: section \"{}\", key \"{}\"", section, key)
	}
}