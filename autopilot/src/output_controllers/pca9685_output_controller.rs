use pwm_pca9685::{Pca9685, Address, Channel};
use rppal::i2c::I2c;
use crate::output::OutputController;
use rppal::hal::Delay;
use std::convert::TryFrom;

pub struct Pca9685OutputController {
	pca9685: Pca9685<I2c>,
	previous_output: [f32; 16],
}

impl Pca9685OutputController {
	pub fn new<A: Into<Address>>(i2c: I2c, address: A) -> anyhow::Result<Self> {
		let mut instance = Self {
			pca9685: Pca9685::new(i2c, address).map_err(|e| anyhow!("{:?}", e))?,
			previous_output: [0f32; 16],
		};

		Ok(instance)
	}

	pub fn init(mut self, prescale: u8) -> anyhow::Result<Self> {
		self.pca9685.set_all_on_off(&[0u16; 16], &[0u16; 16])
			.map_err(|e| anyhow!("{:?}", e))?;

		self.pca9685
			.enable()
			.map_err(|e| anyhow!("{:?}", e))?;

		self.pca9685
			.set_prescale(prescale)
			.map_err(|e| anyhow!("{:?}", e))?;

		Ok(self)
	}

	fn set_channel_on_off_from_output(&mut self, channel: Channel, output: f32) -> anyhow::Result<()> {
		// Outputs are guaranteed to be between -1f32 and 1f32 inclusive.
		const SCALE: f32 = 1. / 2. * 4095.;
		Ok(self.pca9685
			.set_channel_on_off(channel, 0u16, ((output + 1.) * SCALE) as u16)
			.map_err(|e| anyhow!("{:?}", e))?)
	}
}

impl OutputController<[f32; 16]> for Pca9685OutputController {
	type Error = anyhow::Error;

	fn write_output(&mut self, output: [f32; 16]) -> anyhow::Result<()> {
		for (i, &channel_output) in output.iter().enumerate() {
			if channel_output != self.previous_output[i] {
				self.set_channel_on_off_from_output(Channel::try_from(i)
														.map_err(|e| anyhow!("{:?}", e))?,
													channel_output)
					.map_err(|e| anyhow!("{:?}", e))?;
			}
		}

		self.previous_output = output;

		Ok(())
	}
}