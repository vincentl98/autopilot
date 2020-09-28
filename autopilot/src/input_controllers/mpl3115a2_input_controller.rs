use crate::input::{InputController, Input};
use crossbeam_channel::Sender;
use std::time::{Duration, Instant};
use std::{thread};
use rppal::i2c::I2c;
use rppal::hal::Delay;
use mpl3115a2::{Mpl3115A2, PowerMode, SensorMode, OversamplingRatio};

pub struct Mpl3115A2InputController {
	mpl3115a2: Mpl3115A2<I2c>,
}

impl Mpl3115A2InputController {
	pub fn new(i2c: I2c) -> anyhow::Result<Self> {
		let mut mpl3115a2 = Mpl3115A2::new(i2c);
		Ok(Self {
			mpl3115a2,
		})
	}

	pub fn init(mut self) -> anyhow::Result<Self> {
		self.mpl3115a2
			.init(PowerMode::Active,
				  SensorMode::Altitude,
				  OversamplingRatio::Ratio128)
			.map_err(|e| anyhow!("{:?}", e))?;

		const START_DELAY: Duration = Duration::from_millis(2 * 512);

		thread::sleep(START_DELAY); // Guarantees not to fail on reading

		let altitude = self.mpl3115a2
			.read_pa()
			.map_err(|e| anyhow!("{:?}", e))?;

		self.mpl3115a2
			.set_altitude_offset(-altitude as i8)
			.map_err(|e| anyhow!("{:?}", e))?;

		Ok(self)
	}
}

impl InputController for Mpl3115A2InputController {
	// Max data read with oversampling ratio of 2^7 is one every 512 ms
	const DELAY: Option<Duration> = Some(Duration::from_millis(512));

	fn read_input(&mut self) -> anyhow::Result<Input> {
			self.mpl3115a2
				.read_pa()
				.map(|altitude| Input::Altitude(altitude))
				.map_err(|e| anyhow!("mpl3115a2: {:?}", e))
	}
}
