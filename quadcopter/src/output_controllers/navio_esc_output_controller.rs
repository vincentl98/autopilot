use autopilot::OutputController;
use std::error::Error;
use rppal;
use std::io;
use pwm::{PwmPin, Polarity};

pub const QUADCOPTER_ESC_CHANNELS: usize = 4;

pub struct NavioEscOutputController {
	esc_channels: [PwmPin; QUADCOPTER_ESC_CHANNELS],
}

const NAVIO_MAX_PWM_FREQUENCY: u64 = 400;
const NAVIO_PWM_CHIP: u32 = 0;
const PERIOD_NS: u64 = 1_000_000_000u64 / NAVIO_MAX_PWM_FREQUENCY;

const MIN_VALUE_NS: u64 = 1000_000; // 1000 µs
const MAX_VALUE_NS: u64 = 2000_000; // 2000 µs
const VALUE_RANGE_NS: u64 = MAX_VALUE_NS - MIN_VALUE_NS;

impl NavioEscOutputController {
	pub fn new(esc_channels: [u32; 4]) -> io::Result<Self> {
		Ok(Self {
			esc_channels: esc_channels.map(|i| Self::setup_esc_channel(i)
				.expect(&format!("Failed to setup ESC channel {}", i)))
		})
	}

	fn setup_esc_channel(channel: u32) -> Result<PwmPin, io::Error> {
		let pin = PwmPin::new(NAVIO_PWM_CHIP, channel)?;
		pin.export()?;
		pin.set_period(PERIOD_NS)?;
		pin.set_pulse_width(Self::pulse_width_ns(0f32))?;
		pin.set_polarity(Polarity::Normal)?;
		pin.set_enabled(true)?;
		Ok(pin)
	}

	#[inline(always)]
	fn pulse_width_ns(duty_cycle: f32) -> u64 {
		MIN_VALUE_NS + (duty_cycle * VALUE_RANGE_NS as f32).ceil() as u64
	}
}

impl OutputController<[f32; QUADCOPTER_ESC_CHANNELS]> for NavioEscOutputController {
	fn write_output(&mut self, output: [f32; QUADCOPTER_ESC_CHANNELS]) -> Result<(), Box<dyn Error>> {
		for (channel, &value) in self.esc_channels.iter().zip(output.iter()) {
			let pulse_width_ns = Self::pulse_width_ns(value);
			channel.set_pulse_width(pulse_width_ns)?;
		}

		Ok(())
	}
}