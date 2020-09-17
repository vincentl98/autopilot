use rppal::pwm::{Pwm, Polarity};
use crate::output::{OutputController, Output};

pub struct EscOutputController {
	pwm: rppal::pwm::Pwm,
}

impl EscOutputController {
	pub fn new(channel: rppal::pwm::Channel) -> anyhow::Result<Self> {
		let pwm = Pwm::with_frequency(channel,
									  50f64,
									  0.0,
									  Polarity::Normal,
									  true)?;
		Ok(EscOutputController { pwm })
	}
}

impl OutputController for EscOutputController {
	fn f(&self, output: Output) {
		let esc = *output.as_esc().unwrap();
		self.pwm.set_duty_cycle(esc).unwrap();
	}

	fn filter(output: &Output) -> bool {
		match output {
			&Output::Esc(esc) => true,
			_ => false,
		}
	}
}