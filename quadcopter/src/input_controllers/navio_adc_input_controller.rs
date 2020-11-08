/*
The real values of power module voltage and current could be calculated by multiplying to the
respective coefficients. The multiplier for voltage (A2) is 11.3. Coefficient for current (A3)
is 17.0.

For further information see source code. Pay attention to adc.init() and adc.read() functions.
adc.init() function initialize 6 ADC channels. After that it's possible to read value of channels
0-5 with adc.read() function. It takes channel number as an argument.
 */

use std::{io, fs};
use std::error::Error;
use autopilot::{InputController, Input, NavioAdcData};
use systemstat::Duration;

pub struct NavioAdcInputController;

const ADC_PATH: &'static str = "/sys/kernel/rcio/adc";
const CHANNELS: usize = 6;

#[allow(dead_code)]
mod channels {
	pub const BOARD_VOLTAGE: u8 = 0;
	pub const SERVO_VOLTAGE: u8 = 1;
	pub const EXTERNAL_VOLTAGE: u8 = 2;
	pub const EXTERNAL_CURRENT: u8 = 3;
	pub const ADC_PORT_2: u8 = 4;
	pub const ADC_PORT_3: u8 = 5;
}

impl NavioAdcInputController {
	pub fn new() -> Result<Self, io::Error> {
		for i in 0..CHANNELS {
			fs::metadata(format!("{}/ch{}", ADC_PATH, i))?; // Check if files exist
		}

		Ok(Self)
	}

	pub fn read_channels(&self) -> Result<NavioAdcData<f64>, Box<dyn Error>> {
		Ok(NavioAdcData {
			board_voltage: self.read_channel(channels::BOARD_VOLTAGE)?,
			servo_voltage: self.read_channel(channels::SERVO_VOLTAGE)?,
			external_voltage: self.read_channel(channels::EXTERNAL_VOLTAGE)?,
			external_current: self.read_channel(channels::EXTERNAL_CURRENT)?,
			adc_port_2: self.read_channel(channels::ADC_PORT_2)?,
			adc_port_3: self.read_channel(channels::ADC_PORT_3)?,
		})
	}

	pub fn read_channel(&self, channel: u8) -> Result<f64, Box<dyn Error>> {
		let raw_value = Self::channel_parse(channel)? as f64 / 1000.;
		Ok(match channel {
			channels::EXTERNAL_CURRENT => raw_value * 17.0,
			channels::EXTERNAL_VOLTAGE => raw_value * 11.3,
			_ => raw_value
		})
	}

	fn channel_parse(channel: u8) -> Result<u16, Box<dyn Error>> {
		let value = fs::read_to_string(format!("{}/ch{}", ADC_PATH, channel))?;
		Ok(value.trim().parse()?)
	}
}

impl InputController for NavioAdcInputController {
	const DELAY: Option<Duration> = Some(Duration::from_secs(1));

	fn read_input(&mut self) -> Result<Input, Box<dyn Error>> {
		self.read_channels()
			.map(|channels| {
			debug!(target: "adc", "{}", channels);
			channels
		})
			.map(Input::NavioAdc)
	}
}