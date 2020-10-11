use rppal::gpio::{OutputPin, Gpio, Level};
use std::error::Error;
use rppal::gpio::Level::{Low, High};

use crate::quadcopter::LedColor;
use autopilot::OutputController;

pub struct LedOutputController {
	pin_red: OutputPin,
	pin_green: OutputPin,
	pin_blue: OutputPin,
}

impl LedOutputController {
	pub fn new() -> anyhow::Result<Self> {
		let gpio = Gpio::new()?;

		const GPIO_RED: u8 = 4;
		const GPIO_GREEN: u8 = 27;
		const GPIO_BLUE: u8 = 6;

		Ok(Self {
			pin_red: gpio.get(GPIO_RED)?.into_output(),
			pin_green: gpio.get(GPIO_GREEN)?.into_output(),
			pin_blue: gpio.get(GPIO_BLUE)?.into_output(),
		})
	}
}

impl OutputController<Option<LedColor>> for LedOutputController {
	fn write_output(&mut self, output: Option<LedColor>) -> Result<(), Box<dyn Error>> {
		let levels = match output {
			None => (High, High, High),
			Some(color) => {
				match color {
					LedColor::White => (High, High, High),
					LedColor::Green => (High, Low, High),
					LedColor::Blue => (High, High, Low),
					LedColor::Red => (Low, High, High, ),
					_ => {
						warn!("Color not implemented {:?}", color);
						(Low, Low, Low)
					},
				}
			}
		};

		self.pin_red.write(levels.0);
		self.pin_green.write(levels.1);
		self.pin_blue.write(levels.2);

		Ok(())
	}
}