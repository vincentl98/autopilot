#![feature(array_map)]
#![feature(duration_zero)]
#![feature(box_syntax)]

#[macro_use]
extern crate anyhow;

#[macro_use]
extern crate log;

#[macro_use]
extern crate lazy_static;

mod black_box;
mod input;
mod input_controllers;
mod monitors;
mod output_controllers;
mod tank;
mod traits;

use crate::{
    black_box::BlackBoxController,
    input::Input,
    input_controllers::{
        bno055_input_controller::Bno055InputController, sbus_input_controller::SbusInputController,
        soft_arm_input_controller::SoftArmInputController,
    },
    monitors::system_information_monitor::SystemInformationMonitor,
    output_controllers::l298n_output_controller::{L298NOutputController, MotorPins},
    tank::{TankAutopilot, TankCollector, TankInputFrame, TankOutputFrame},
    traits::{Autopilot, Collector, Dispatcher, InputController, Monitor, OutputController},
};
use crossbeam_channel::unbounded;
use log::LevelFilter;
use rppal::i2c::I2c;
use std::{thread, time::Duration};
use tini::Ini;

fn main() {
    std::env::set_var("RUST_BACKTRACE", "full");

    info!("Autopilot {}", env!("CARGO_PKG_VERSION"));

    let (level_filter, pid, _prescale) = read_config().expect("Failed to read configuration file");

    let black_box = BlackBoxController::new();
    black_box.spawn(level_filter);

    let l298n_output_controller = L298NOutputController::new(
        MotorPins {
            pwm_channel: rppal::pwm::Channel::Pwm0,
            pin_in_1: 17,
            pin_in_2: 27,
        },
        MotorPins {
            pwm_channel: rppal::pwm::Channel::Pwm1,
            pin_in_1: 5,
            pin_in_2: 6,
        },
    )
    .expect("Failed to create L298N");

    let armed_input_controller = SoftArmInputController::new();
    let armed_sender = armed_input_controller.sender();

    // Output controllers
    let (motors_sender, motors_receiver) = unbounded::<(f64, f64)>();
    l298n_output_controller.spawn(motors_receiver);

    // Dispatcher
    let (output_frame_sender, output_frame_receiver) = unbounded::<TankOutputFrame>();
    let dispatcher = tank::TankDispatcher { motors_sender };

    dispatcher.spawn(output_frame_receiver);

    // Autopilot
    let (input_frame_sender, input_frame_receiver) = unbounded::<TankInputFrame>();
    TankAutopilot::new(pid).spawn(input_frame_receiver, output_frame_sender);

    // Collector
    let (input_sender, input_receiver) = unbounded::<Input>();

    let collector = TankCollector::new();
    collector.spawn(input_receiver, input_frame_sender);

    // Input controllers
    Bno055InputController::new(I2c::with_bus(3).unwrap())
        .unwrap()
        .init(None)
        .unwrap()
        .spawn(input_sender.clone());

    SystemInformationMonitor::new().spawn();

    SbusInputController::new()
        .unwrap()
        .spawn(input_sender.clone());

    armed_input_controller.spawn(input_sender.clone());

    armed_sender.send(true).unwrap();

    println!("Press enter to stop autopilot ...");
    std::io::stdin().read_line(&mut String::new()).unwrap();

    armed_sender.send(false).unwrap();

    thread::sleep(Duration::from_millis(100));
}

fn read_config() -> anyhow::Result<(LevelFilter, (f32, f32, f32), u8)> {
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

    const PCA9685_SECTION: &'static str = "pca_9685";
    const PCA9685_PRESCALE: &'static str = "prescale";
    let prescale: u8 = config.get(PCA9685_SECTION, PCA9685_PRESCALE).unwrap_or(100);

    Ok((level_filter, pid, prescale))
}
