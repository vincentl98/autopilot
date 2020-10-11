use crossbeam_channel::{unbounded, Receiver, Sender};
use std::{error::Error, time::Duration};
use autopilot::{InputController, Input};


/// Controls the arming of the autopilot. Used only in autopilots using an arm/disarm mechanism.
/// Sending `true` through the channel will arm the vehicle.
pub struct SoftArmInputController {
	sender: Sender<bool>,
	receiver: Receiver<bool>,
}

impl SoftArmInputController {
	pub fn new() -> Self {
		let (sender, receiver) = unbounded::<bool>();
		SoftArmInputController { sender, receiver }
	}

	pub fn sender(&self) -> Sender<bool> {
		self.sender.clone()
	}
}

impl InputController for SoftArmInputController {
	const DELAY: Option<Duration> = None;

	fn read_input(&mut self) -> Result<Input, Box<dyn Error>> {
		Ok(Input::SoftArmed(self.receiver.recv()?))
	}
}
