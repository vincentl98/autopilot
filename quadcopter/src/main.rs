#![feature(array_map)]

#[macro_use]
extern crate anyhow;

#[macro_use]
extern crate log;

use autopilot::{Input, Dispatcher, Autopilot, Collector, InputController, OutputController, Monitor};

mod input_controllers;
mod monitors;
mod output_controllers;
mod quadcopter;
mod quadcopter_autopilot;
mod quadcopter_config;

use crate::{
	input_controllers::{
		soft_arm_input_controller::SoftArmInputController,
	},
	monitors::system_information_monitor::SystemInformationMonitor,
};
use black_box::BlackBox;
use crossbeam_channel::unbounded;
use log::LevelFilter;
use std::{thread, time::Duration};
use tini::Ini;

use crate::quadcopter::{QuadcopterOutputFrame, QuadcopterInputFrame, QuadcopterCollector, LedColor};
use crate::output_controllers::led_output_controller::LedOutputController;
use crate::input_controllers::lsm9ds1_input_controller::LSM9DS1InputController;
use crate::input_controllers::navio_adc_input_controller::NavioAdcInputController;
use crate::input_controllers::navio_rc_input_controller::{NavioRcInputController};
use crate::output_controllers::navio_esc_output_controller::{NavioEscOutputController, QUADCOPTER_ESC_CHANNELS};
use crate::quadcopter_autopilot::QuadcopterAutopilot;
use std::convert::TryFrom;
use ahrs::{Madgwick, Mahony};
use nalgebra::Vector3;

fn main() {
	const CONFIG_FILE_PATH: &'static str = "config.ini";
	let config_ini = Ini::from_file(CONFIG_FILE_PATH).unwrap();

	let config = quadcopter_config::QuadcopterConfig::try_from(&config_ini).unwrap();

	let black_box = BlackBox::new();
	black_box.spawn(config.log_level_filter);

	info!("Autopilot {}", env!("CARGO_PKG_VERSION"));

	let armed_input_controller = SoftArmInputController::new();
	let armed_sender = armed_input_controller.sender();

	// Output controllers
	let (led_sender, led_receiver) = unbounded::<Option<LedColor>>();
	LedOutputController::new().unwrap().spawn(led_receiver);

	let (esc_channels_sender, esc_channels_receiver) = unbounded::<[f64; QUADCOPTER_ESC_CHANNELS]>();
	NavioEscOutputController::new(config.output_esc_pins)
		.init()
		.unwrap()
		.spawn(esc_channels_receiver);

	// Dispatcher
	let (output_frame_sender, output_frame_receiver) = unbounded::<QuadcopterOutputFrame>();
	let dispatcher = quadcopter::QuadcopterDispatcher { led_sender, esc_channels_sender };

	dispatcher.spawn(output_frame_receiver);

	// Autopilot
	let (input_frame_sender, input_frame_receiver) = unbounded::<QuadcopterInputFrame>();
	QuadcopterAutopilot::new(config.pid_roll,
							 config.pid_pitch,
							 config.pid_yaw).spawn(input_frame_receiver, output_frame_sender);

	// Collector
	let (input_sender, input_receiver) = unbounded::<Input>();

	let collector = QuadcopterCollector::new();
	collector.spawn(input_receiver, input_frame_sender);

	// Input controllers
	const NAVIO2_ACC_GYR_PATH: &'static str = "/dev/spidev0.3";
	const NAVIO2_MAG_PATH: &'static str = "/dev/spidev0.2";

	let mut lsm9ds1 = LSM9DS1InputController::new(
		NAVIO2_ACC_GYR_PATH,
		NAVIO2_MAG_PATH,
		config.filter_accelerometer,
		config.filter_gyroscope,
		Mahony::<f64>::new(0.5, 0.0),
	).unwrap();

	let flat_trim = {
		let args: Vec<String> = std::env::args().collect();
		if args.len() > 1 && args[1] == "--flat-trim" {
			true
		} else {
			false
		}
	};

	if flat_trim {
		info!("Performing flat trim calibration");

		let (acc_offset, gyr_offset) = lsm9ds1.calibrate().unwrap();

		lsm9ds1.set_calibration(acc_offset.clone().into(),
								gyr_offset.clone().into());

		config_ini
			.section("offset")
			.item_vec("acc", &[acc_offset.x, acc_offset.y, acc_offset.z])
			.item_vec("gyr", &[gyr_offset.x, gyr_offset.y, gyr_offset.z])
			.to_file(CONFIG_FILE_PATH).unwrap();
	} else {
		info!("Using previously saved calibration");
		lsm9ds1.set_calibration(config.offset_acc, config.offset_gyr);
	}

	lsm9ds1.spawn(input_sender.clone());

	NavioAdcInputController::new().unwrap().spawn(input_sender.clone());

	NavioRcInputController::new(config.input_rc_range).unwrap().spawn(input_sender.clone());

	armed_input_controller.spawn(input_sender.clone());

	// Monitors
	SystemInformationMonitor::new().spawn();

	armed_sender.send(true).unwrap();

	info!("Press enter to stop autopilot");
	std::io::stdin().read_line(&mut String::new()).unwrap();

	armed_sender.send(false).unwrap();

	thread::sleep(Duration::from_millis(100));
}