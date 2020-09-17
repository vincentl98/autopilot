use crossbeam_channel::{Receiver};
use std::thread::JoinHandle;
use std::thread;


pub trait OutputController<T: Send + 'static>
	where Self: Sized + Send + 'static {

	/// Writes an output value to a physical actuator.
	fn write_output(&mut self, output: T) -> anyhow::Result<()>;

	fn spawn(mut self, output_receiver: Receiver<T>) -> JoinHandle<()> {
		thread::spawn(move || {
			for output in output_receiver.iter() {
				self.write_output(output)
					.map_err(|e| error!("Failed to write output: {}", e))
					.unwrap_or_default();
			}
		})
	}
}