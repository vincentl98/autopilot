use crate::output::{OutputFrame, Output};
use crate::autopilot::Autopilot;
use crate::input::InputFrame;

pub struct FixedWindOutputFrame {
	pub esc: f64,
	pub left_aileron: f64,
	pub right_aileron: f64,
	pub elevator: f64,
	pub rudder: f64,
}

impl OutputFrame for FixedWindOutputFrame {}

impl Into<Vec<Output>> for FixedWindOutputFrame {
	fn into(self) -> Vec<Output> {
		vec![
			Output::Esc(self.esc),
			Output::LeftAileron(self.left_aileron),
			Output::RightAileron(self.right_aileron),
			Output::Elevator(self.elevator),
			Output::Rudder(self.rudder),
		]
	}
}


/// Flight controller for fixed wing aircraft.
#[derive(Copy, Clone)]
pub struct FixedWingAutopilot;

impl Autopilot for FixedWingAutopilot {
	type Frame = FixedWindOutputFrame;

	fn control(&self, input_frame: InputFrame) -> FixedWindOutputFrame {
		if let Some(rc_channels) = input_frame.rc_channels {
			// println!("got rc_channels : {:?}", &rc_channels);
			FixedWindOutputFrame {
				esc: (rc_channels.channels[3] + 1.0) / 2.0,
				left_aileron: -rc_channels.channels[0],
				right_aileron: rc_channels.channels[0],
				elevator: rc_channels.channels[1],
				rudder: rc_channels.channels[4],
			}
		} else {
			FixedWindOutputFrame {
				esc: 0.0,
				left_aileron: 0.5,
				right_aileron: 0.5,
				elevator: 0.5,
				rudder: 0.5,
			}
		}
	}
}