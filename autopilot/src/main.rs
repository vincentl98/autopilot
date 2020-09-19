#![feature(array_map)]

#[macro_use]
extern crate anyhow;

#[macro_use]
extern crate log;

#[macro_use]
extern crate lazy_static;

mod input_controllers;

mod output_controllers;

mod autopilot;
mod black_box;
mod collector;
mod dispatcher;
mod input;
mod output;
mod tank;

use crossbeam_channel::unbounded;
use dispatcher::Dispatcher;
use log::LevelFilter;
use std::thread;
use std::time::Duration;
use tini::Ini;

use crate::autopilot::Autopilot;
use crate::black_box::BlackBoxController;
use crate::collector::{Collector, InputFrame};
use crate::input::{Input, InputController};
use crate::input_controllers::armed_input_controller::SoftArmedInputController;
use crate::input_controllers::bno055_input_controller::Bno055InputController;
use crate::input_controllers::sbus_input_controller::SbusInputController;
use crate::input_controllers::system_information_input_controller::SystemInformationInputController;
use crate::output::OutputController;
use crate::output_controllers::l298n_output_controller::{L298NOutputController, MotorPins};
use crate::tank::{TankAutopilot, TankOutputFrame};
use bno055::BNO055OperationMode;
use rppal::i2c::I2c;

fn main() {
	std::env::set_var("RUST_BACKTRACE", "full");

	println!("Autopilot {}", env!("CARGO_PKG_VERSION"));

	let (level_filter, pid) = read_config().expect("Failed to read configuration file");

	let black_box = BlackBoxController::new();
	black_box.spawn(level_filter);

	//
	// let l298n_output_controller = L298NOutputController::new(
	//     MotorPins {
	//         pwm_channel: rppal::pwm::Channel::Pwm0,
	//         pin_in_1: 17,
	//         pin_in_2: 27,
	//     },
	//     MotorPins {
	//         pwm_channel: rppal::pwm::Channel::Pwm1,
	//         pin_in_1: 5,
	//         pin_in_2: 6,
	//     },
	// )
	// .expect("Failed to create L298N");

	let armed_input_controller = SoftArmedInputController::new();
	let armed_sender = armed_input_controller.sender();

	let car_autopilot = TankAutopilot::new(pid);

	// Output controllers
	// let (motors_sender, motors_receiver) = unbounded::<(f64, f64)>();
	//
	// l298n_output_controller.spawn(motors_receiver);

	// // Dispatcher
	let (output_frame_sender, output_frame_receiver) = unbounded::<TankOutputFrame>();
	// let dispatcher = tank::TankDispatcher { motors_sender };
	//
	// dispatcher.spawn(output_frame_receiver);

	// Autopilot
	let (input_frame_sender, input_frame_receiver) = unbounded::<InputFrame>();
	car_autopilot.spawn(input_frame_receiver, output_frame_sender);

	// Collector
	let (input_sender, input_receiver) = unbounded::<Input>();

	let collector = tank::TankCollector::new();
	collector.spawn(input_receiver, input_frame_sender);

	// Input controllers
	let i2c_3 = I2c::with_bus(3).unwrap();

	Bno055InputController::new(i2c_3, None)
		.expect("Failed to initialize BNO055")
		.spawn(input_sender.clone());

	SystemInformationInputController::new().spawn(input_sender.clone());

	SbusInputController::new()
	    .unwrap()
	    .spawn(input_sender.clone());

	armed_input_controller.spawn(input_sender.clone());

	armed_sender.send(true).unwrap();

	println!("Press enter to stop autopilot ...");
	std::io::stdin()
		.read_line(&mut String::new())
		.expect("Failed to read standard input");

	armed_sender.send(false).unwrap();

	println!("Stopping ...");
	thread::sleep(Duration::from_millis(100));
}

fn read_config() -> anyhow::Result<(LevelFilter, (f32, f32, f32))> {
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

	const PID_HEADING_SECTION: &'static str = "pid";
	const PID_P: &'static str = "p";
	const PID_I: &'static str = "i";
	const PID_D: &'static str = "d";
	let pid: (f32, f32, f32) = (
		config.get(PID_HEADING_SECTION, PID_P).unwrap(),
		config.get(PID_HEADING_SECTION, PID_I).unwrap(),
		config.get(PID_HEADING_SECTION, PID_D).unwrap(),
	);

	Ok((level_filter, pid))
}
