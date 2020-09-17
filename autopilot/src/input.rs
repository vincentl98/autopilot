use crossbeam_channel::Sender;
use std::thread::JoinHandle;
use std::thread;
use crate::input_controllers::system_information_input_controller::SystemInformation;

const MAX_RC_CHANNELS: usize = 16;

#[derive(Default, Debug, Copy, Clone)]
pub struct RcChannels {
	pub channels: [f32; MAX_RC_CHANNELS],
	pub failsafe: bool,
}

/// Abstraction of a physical measurement.
/// Bounded values such as RC channels should be normalized by the input controller beforehand.
#[derive(Copy, Clone, Debug)]
pub enum Input {
	SystemInformation(SystemInformation),
	// BatteryVoltage(f64),
	// Current(f64),
	// Pressure(f64),
	// Position ([f64; 3]),
	OrientationEuler(mint::EulerAngles<f32, ()>),
	// Velocity([f64; 3]),
	// AngularVelocity([f64; 3]),
	// Acceleration([f64; 3]),
	RcChannels(RcChannels),
	Armed(bool),
}

pub trait InputController
	where Self: Sized + Send + 'static {
	/// Reads a physical value from a sensor or an input device.
	fn read_input(&mut self, input_sender: Sender<Input>) -> !;
	fn spawn(mut self, input_sender: Sender<Input>) -> JoinHandle<()> {
		thread::spawn(move || self.read_input(input_sender))
	}
}