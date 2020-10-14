// Adapted from libraries `rppal` by Rene van der Meer and `sysfs-pwm` by the Rust Embedded team

use std::{fs, io};
use std::fs::File;
use std::io::prelude::*;
use std::path::Path;
use std::time::Duration;
use std::error::Error;

#[derive(Debug)]
pub struct PwmChip {
	pub number: u32,
}

impl PwmChip {
	pub fn new(number: u32) -> io::Result<PwmChip> {
		fs::metadata(&format!("/sys/class/pwm/pwmchip{}", number))?;
		Ok(PwmChip { number })
	}

	pub fn count(&self) -> Result<u32, Box<dyn Error>> {
		let npwm = fs::read_to_string(format!("/sys/class/pwm/pwmchip{}/npwm", self.number))?;
		Ok(npwm.trim().parse()?)
	}
}

#[derive(Debug)]
pub enum Polarity {
	Normal,
	Inverse,
}

#[derive(Debug)]
pub struct PwmPin {
	chip: PwmChip,
	channel: u32,
	unexport_on_drop: bool,
}

const PWM_PATH: &'static str = "/sys/class/pwm";

impl PwmPin {
	pub fn new(chip: u32, channel: u32) -> io::Result<PwmPin> {
		let chip: PwmChip = PwmChip::new(chip)?;

		Ok(PwmPin {
			chip,
			channel,
			unexport_on_drop: true,
		})
	}


	pub fn export(&self) -> io::Result<()> {
		// Exporting a PWM pin takes a significant time, that is also unpredictable.
		let pwm_path = Path::new(&format!("{}/pwmchip{}/pwm{}", PWM_PATH, self.chip.number, self.channel)).to_owned();
		if !pwm_path.exists() {
			File::create(format!("{}/pwmchip{}/export", PWM_PATH, self.chip.number))?
				.write_fmt(format_args!("{}", self.channel))?;

			const MAX_RETRY: usize = 25;
			for _ in 0..MAX_RETRY {
				if !pwm_path.exists() {
					const EXPORT_DELAY: Duration = Duration::from_millis(30);
					std::thread::sleep(EXPORT_DELAY);
				} else {
					break;
				}
			}
		}

		fs::metadata(pwm_path).map(|_| ())
	}

	pub fn unexport(&self) -> io::Result<()> {
		// Only unexport if the channel is actually exported
		if Path::new(&format!("{}/pwmchip{}/pwm{}", PWM_PATH, self.chip.number, self.channel)).exists() {
			File::create(format!("{}/pwmchip{}/unexport", PWM_PATH, self.chip.number))?
				.write_fmt(format_args!("{}", self.channel))?;
		}

		Ok(())
	}

	pub fn period(&self) -> io::Result<u64> {
		let period = fs::read_to_string(format!("{}/pwmchip{}/pwm{}/period", PWM_PATH, self.chip.number, self.channel))?;
		Ok(period.trim().parse()?)
	}

	pub fn set_period(&mut self, period: u64) -> io::Result<()> {
		File::create(format!("{}/pwmchip{}/pwm{}/period", PWM_PATH, self.chip.number, self.channel))?
			.write_fmt(format_args!("{}", period))?;

		Ok(())
	}

	pub fn pulse_width(self) -> Result<u64, Box<dyn Error>> {
		// The sysfs PWM interface specifies the duty cycle in nanoseconds, which
		// means it's actually the pulse width.
		let duty_cycle =
			fs::read_to_string(format!("{}/pwmchip{}/pwm{}/duty_cycle", PWM_PATH, self.chip.numer, self.channel))?;

		Ok(duty_cycle.trim().parse()?)
	}

	pub fn set_pulse_width(&mut self, pulse_width_ns: u64) -> io::Result<()> {
		File::create(format!("{}/pwmchip{}/pwm{}/duty_cycle", PWM_PATH, self.chip.number, self.channel))?
			.write_fmt(format_args!("{}", pulse_width_ns))?;

		Ok(())
	}

	pub fn polarity(&self) -> io::Result<Polarity> {
		let polarity = fs::read_to_string(format!("{}/pwmchip{}/pwm{}/polarity", PWM_PATH, self.chip.number, self.channel))?;

		match polarity.trim() {
			"normal" => Ok(Polarity::Normal),
			_ => Ok(Polarity::Inverse),
		}
	}

	pub fn set_polarity(&mut self, polarity: Polarity) -> io::Result<()> {
		let b_polarity: &[u8] = match polarity {
			Polarity::Normal => b"normal",
			Polarity::Inverse => b"inversed",
		};

		File::create(format!("{}/pwmchip{}/pwm{}/polarity", PWM_PATH, self.chip.number, self.channel))?
			.write_all(b_polarity)?;

		Ok(())
	}

	pub fn enabled(&self) -> io::Result<bool> {
		let enabled = fs::read_to_string(format!("{}/pwmchip{}/pwm{}/enable", PWM_PATH,
												 self.chip.number, self.channel))?;

		match enabled.trim() {
			"1" => Ok(true),
			_ => Ok(false),
		}
	}

	pub fn set_enabled(&mut self, enabled: bool) -> io::Result<()> {
		File::create(format!("{}/pwmchip{}/pwm{}/enable", PWM_PATH, self.chip.number, self.channel))?
			.write_fmt(format_args!("{}", enabled as u8))?;

		Ok(())
	}
}

impl Drop for PwmPin {
	fn drop(&mut self) {
		if self.unexport_on_drop {
			self.unexport().unwrap_or_default()
		}
	}
}