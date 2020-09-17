use crate::output::{OutputController, Output};

/// TODO: use existing library
struct Pca9685OutputController;
impl OutputController for Pca9685OutputController {
	fn write(&mut self, output: Output) {
		unimplemented!()
	}

	fn filter(output: &Output) -> bool {
		match output {
			&Output::Esc(_) => true,
			&Output::LeftAileron(_) => true,
			&Output::RightAileron(_) => true,
			&Output::Rudder(_) => true,
			&Output::Elevator(_) => true,
			_ => false,
		}
	}

	fn reset(&mut self) {
		unimplemented!()
	}
}