use crate::input::{InputController, Input};
use crossbeam_channel::Sender;
use std::time::Duration;
use std::{thread};
use bno055::Bno055;
use rppal::i2c::I2c;

pub struct Bno055InputController {
	bno055: Bno055<I2c>
}

impl Bno055InputController {
	pub fn new() -> anyhow::Result<Self> {
		let i2c = I2c::new()?;
		Ok(Self {
			bno055: Bno055::<I2c>::new(i2c)
		})
	}
}

impl InputController for Bno055InputController {
	fn read_input(&mut self, input_sender: Sender<Input>) -> ! {
		self.bno055
			.init(&mut rppal::hal::Delay::new())
			.expect("Failed to initialize BNO055 sensor");
		loop {
			match self.bno055.euler_angles() {
				Ok(euler_angles) => {
					input_sender
						.send(Input::OrientationEuler(euler_angles))
						.map_err(|e| error!("{}", e))
						.unwrap_or_default()
				},
				Err(e) => error!("Failed to read sensor data: {:?}", e)
			}

			const INPUT_LOOP_DELAY: Duration = Duration::from_millis(100);
			thread::sleep(INPUT_LOOP_DELAY);
		}
	}
}
