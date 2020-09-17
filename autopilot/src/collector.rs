use crossbeam_channel::{Receiver, Sender};
use std::thread::JoinHandle;
use std::thread;
use crate::input::{Input, RcChannels};
use std::time::Instant;
use crate::input_controllers::system_information_input_controller::SystemInformation;

/// Buffers inputs sent asynchronously by input controllers.
#[derive(Debug, Default, Clone)]
pub struct InputFrame {
	pub system_information: Option<SystemInformation>,
	pub rc_channels: Option<RcChannels>,
	pub armed: Option<bool>,
	pub orientation: Option<(mint::EulerAngles<f32, ()>, Instant)>,
}

pub trait Collector
	where Self: Sized + Send + 'static {
	fn collect(&mut self, input: Input) -> InputFrame;

	fn spawn(mut self,
			 input_receiver: Receiver<Input>,
			 input_frame_sender: Sender<InputFrame>) -> JoinHandle<()> {

		thread::spawn(move || {
			for input in input_receiver {
				let input_frame = self.collect(input);
				trace!("{:?}", &input_frame);

				if let Err(e) = input_frame_sender.send(input_frame) {
					error!("{}", e);
				}
			}
		})
	}
}