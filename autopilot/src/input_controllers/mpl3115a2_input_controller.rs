use crate::input::{InputController, Input};
use crossbeam_channel::Sender;
use std::time::{Duration, Instant};
use std::{thread};
use rppal::i2c::I2c;
use rppal::hal::Delay;
use mpl3115a2::{Mpl3115A2, PowerMode, SensorMode, OversamplingRatio};

// Max data read with oversampling ratio of 2^7 is one every 512 ms
const DATA_OUTPUT_RATE_DELAY: Duration = Duration::from_millis(512);

pub struct Mpl3115A2InputController {
	mpl3115a2: Mpl3115A2<I2c>,
	altitude_offset: Option<f32>,
}

impl Mpl3115A2InputController {
	pub fn new(i2c: I2c) -> anyhow::Result<Self> {
		let mut mpl3115a2 = Mpl3115A2::new(i2c);

		mpl3115a2
			.init(PowerMode::Active,
				  SensorMode::Altitude,
				  OversamplingRatio::Ratio128)
			.map_err(|e| anyhow!("{:?}", e))?;

		Ok(Self {
			mpl3115a2,
			altitude_offset: None,
		})
	}
}

impl InputController for Mpl3115A2InputController {
	fn read_input(&mut self, input_sender: Sender<Input>) -> ! {
		loop {
			match self.mpl3115a2.read_pa_temperature() {
				Ok((altitude, temperature)) => {
					if let Some(altitude_offset) = self.altitude_offset {
						input_sender
							.send(Input::Altitude(altitude - altitude_offset))
							.map_err(|e| error!("{}", e))
							.unwrap_or_default();
					} else {
						self.altitude_offset = Some(altitude);
						input_sender
							.send(Input::Altitude(altitude))
							.map_err(|e| error!("{}", e))
							.unwrap_or_default();
					}

					input_sender
						.send(Input::Temperature(temperature))
						.map_err(|e| error!("{}", e))
						.unwrap_or_default();
				}
				Err(e) => error!("{:?}", e)
			}

			thread::sleep(DATA_OUTPUT_RATE_DELAY);
		}
	}
}
