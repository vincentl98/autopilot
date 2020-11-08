#![feature(array_map)]

#[macro_use]
extern crate anyhow;

#[macro_use]
extern crate log;

use crossbeam_channel::unbounded;
use nalgebra::Vector3;

use crate::config::TryIntoLevelFilter;
use crate::input_controllers::soft_arm_input_controller::SoftArmInputController;
use crate::input_controllers::navio_adc_input_controller::NavioAdcInputController;
use crate::monitors::system_information_monitor::SystemInformationMonitor;
use crate::input_controllers::navio_rc_input_controller::NavioRcInputController;
use crate::quadcopter::{LedColor, QuadcopterOutputFrame, QuadcopterInputFrame, QuadcopterCollector};
use crate::output_controllers::led_output_controller::LedOutputController;
use crate::output_controllers::navio_esc_output_controller::{QUADCOPTER_ESC_CHANNELS, NavioEscOutputController};
use crate::quadcopter_autopilot::QuadcopterAutopilot;
use crate::input_controllers::lsm9ds1_input_controller::LSM9DS1InputController;

use autopilot::*;
use ahrs::Madgwick;
use black_box::BlackBox;

mod input_controllers;
mod monitors;
mod output_controllers;
mod quadcopter;
mod quadcopter_autopilot;
mod config;
mod roll_pitch_yaw;
mod mixer;

fn main() {
	let mut config = config::read().unwrap();

	let black_box = BlackBox::new();
	black_box.spawn(config.log_level_filter
		.try_into_level_filter()
		.unwrap());

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
	QuadcopterAutopilot::new(config.pid_values, config.rates, config.limits)
		.spawn(input_frame_receiver, output_frame_sender);

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
		Madgwick::<f64>::new(config.ahrs_madgwick_beta),
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

		lsm9ds1.set_calibration(acc_offset.clone(),
								gyr_offset.clone());

		config.calibration_acc = [acc_offset.x, acc_offset.y, acc_offset.z];
		config.calibration_gyr = [gyr_offset.x, gyr_offset.y, gyr_offset.z];

		config::save(&config).unwrap();
	} else {
		info!("Using previously saved calibration");
		let acc_offset = Vector3::new(config.calibration_acc[0],
									  config.calibration_acc[1],
									  config.calibration_acc[2]);

		let gyr_offset = Vector3::new(config.calibration_gyr[0],
									  config.calibration_gyr[1],
									  config.calibration_gyr[2]);

		lsm9ds1.set_calibration(acc_offset, gyr_offset);
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

	std::thread::sleep(std::time::Duration::from_millis(100));
}