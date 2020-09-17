use std::thread::JoinHandle;
use std::thread;
use crossbeam_channel::{Receiver};
use std::fmt::Debug;

pub trait OutputFrame: Debug {}

pub trait Dispatcher<T: OutputFrame + Send + Sized + 'static>
	where Self: Sized + Send + 'static {

	fn dispatch(&self, output_frame: T) -> anyhow::Result<()>;

	fn dispatch_loop(&self, output_frame_receiver: Receiver<T>) {
		for output_frame in output_frame_receiver {
			trace!("{:?}", &output_frame);
			if let Err(e) = self.dispatch(output_frame) {
				error!("{}", e);
			}
		}
	}

	fn spawn(self, output_frame_receiver: Receiver<T>) -> JoinHandle<()> {
		thread::spawn(move || self.dispatch_loop(output_frame_receiver))
	}
}