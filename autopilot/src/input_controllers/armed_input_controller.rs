use crossbeam_channel::{Receiver, Sender, unbounded};
use crate::input::{InputController, Input};
use log::{error};

/// Controls the arming of the autopilot. Used only in autopilots using an arm/disarm mechanism.
/// Sending `true` through the channel will arm the vehicle.
pub struct SoftArmedInputController {
	sender: Sender<bool>,
	receiver: Receiver<bool>,
}

impl SoftArmedInputController {
	pub fn new() -> Self {
		let (sender, receiver) = unbounded::<bool>();
		SoftArmedInputController {
			sender,
			receiver,
		}
	}

	pub fn sender(&self) -> Sender<bool> {
		self.sender.clone()
	}
}

impl InputController for SoftArmedInputController {
	fn read_input(&mut self, input_sender: Sender<Input>) -> ! {
		loop {
			for armed in self.receiver.clone() {
				input_sender
					.send(Input::SoftArmed(armed))
					.map_err(|e| error!("{}", e))
					.unwrap()
			}
		}
	}
}
