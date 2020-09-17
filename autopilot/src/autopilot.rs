use crossbeam_channel::{Sender, Receiver};
use std::thread::JoinHandle;
use std::thread;
use std::time::{Instant, Duration};
use crate::dispatcher::OutputFrame;
use crate::collector::InputFrame;

pub trait Autopilot
	where Self: Sized + Send + Sync + 'static {

	type Frame: OutputFrame + Send;

	fn output_frame(&mut self, input_frame: InputFrame) -> Self::Frame;

	fn spawn(mut self,
			 input_frame_receiver: Receiver<InputFrame>,
			 output_frame_sender: Sender<Self::Frame>) -> JoinHandle<()> {

		thread::spawn(move || {
			const MAX_OUTPUT_FRAME_DELAY: Duration = Duration::from_millis(50);
			let mut last_output_frame_instant = Instant::now();

			while let Ok(input_frame) = input_frame_receiver.recv() {
				if input_frame_receiver.is_empty() ||
					(Instant::now() - last_output_frame_instant) >= MAX_OUTPUT_FRAME_DELAY {

					let output_frame = self.output_frame(input_frame);
					&output_frame_sender
						.send(output_frame)
						.expect("Failed to send output frame through the channel");
					last_output_frame_instant = Instant::now();
				}
			}
		})
	}
}