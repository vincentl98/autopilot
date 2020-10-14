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

use crate::{
	input_controllers::{
		soft_arm_input_controller::SoftArmInputController,
	},
	monitors::system_information_monitor::SystemInformationMonitor,
};
use crossbeam_channel::unbounded;
use log::LevelFilter;
use std::{thread, time::Duration};
use tini::Ini;
use crate::quadcopter::{QuadcopterOutputFrame, QuadcopterInputFrame, QuadcopterAutopilot, QuadcopterCollector, LedColor};
use crate::output_controllers::led_output_controller::LedOutputController;
use crate::input_controllers::lsm9ds1_input_controller::LSM9DS1InputController;
use black_box::BlackBox;
use crate::input_controllers::navio_adc_input_controller::NavioAdcInputController;
use crate::input_controllers::navio_rc_input_controller::{NavioRcInputController, FLYSKY_RANGE};
use std::convert::TryInto;
use crate::output_controllers::navio_esc_output_controller::{NavioEscOutputController, QUADCOPTER_ESC_CHANNELS};

fn main() {
	std::env::set_var("RUST_BACKTRACE", "full");

	info!("Autopilot {}", env!("CARGO_PKG_VERSION"));

	let (level_filter) = read_config().expect("Failed to read configuration file");

	let black_box = BlackBox::new();
	black_box.spawn(level_filter);

	let armed_input_controller = SoftArmInputController::new();
	let armed_sender = armed_input_controller.sender();

	// Output controllers
	let (led_sender, led_receiver) = unbounded::<Option<LedColor>>();
	LedOutputController::new().unwrap().spawn(led_receiver);

	let (esc_channels_sender, esc_channels_receiver) = unbounded::<[f32; QUADCOPTER_ESC_CHANNELS]>();
	NavioEscOutputController::new([0, 1, 2, 3]).unwrap().spawn(esc_channels_receiver);

	// Dispatcher
	let (output_frame_sender, output_frame_receiver) = unbounded::<QuadcopterOutputFrame>();
	let dispatcher = quadcopter::QuadcopterDispatcher { led_sender, esc_channels_sender };

	dispatcher.spawn(output_frame_receiver);

	// Autopilot
	let (input_frame_sender, input_frame_receiver) = unbounded::<QuadcopterInputFrame>();
	QuadcopterAutopilot::new(1.5f32).spawn(input_frame_receiver, output_frame_sender);

	// Collector
	let (input_sender, input_receiver) = unbounded::<Input>();

	let collector = QuadcopterCollector::new();
	collector.spawn(input_receiver, input_frame_sender);

	// Input controllers
	const NAVIO2_ACC_GYR_PATH: &'static str = "/dev/spidev0.3";
	const NAVIO2_MAG_PATH: &'static str = "/dev/spidev0.2";

	let mut lsm9ds1 = LSM9DS1InputController::new(
		NAVIO2_ACC_GYR_PATH,
		NAVIO2_MAG_PATH).unwrap();
	lsm9ds1.calibrate().unwrap();
	lsm9ds1.spawn(input_sender.clone());

	NavioAdcInputController::new().unwrap().spawn(input_sender.clone());

	NavioRcInputController::new(FLYSKY_RANGE).unwrap().spawn(input_sender.clone());

	armed_input_controller.spawn(input_sender.clone());

	// Monitors
	SystemInformationMonitor::new().spawn();

	armed_sender.send(true).unwrap();

	println!("Press enter to stop autopilot ...");
	std::io::stdin().read_line(&mut String::new()).unwrap();

	armed_sender.send(false).unwrap();

	thread::sleep(Duration::from_millis(100));
}

fn read_config() -> anyhow::Result<(LevelFilter)> {
	const CONFIG_FILE_NAME: &'static str = "autopilot.ini";

	let config = Ini::from_file(CONFIG_FILE_NAME).expect("Failed to load configuration file");

	const LOG_SECTION: &'static str = "log";
	const LEVEL_FILTER: &'static str = "level";

	let level_filter = match config
		.get::<String>(LOG_SECTION, LEVEL_FILTER)
		.unwrap()
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

	Ok((level_filter))
}