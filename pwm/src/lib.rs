// Adapted from libraries `rppal` by Rene van der Meer and `sysfs-pwm` by the Rust Embedded team

use std::io;

mod sysfs;

#[derive(Debug)]
pub enum Polarity {
	Normal,
	Inverse,
}

#[derive(Debug)]
pub struct PwmPin {
	channel: u32,
	unexport_on_drop: bool,
}


impl PwmPin {
	pub fn new(channel: u32) -> PwmPin {
		PwmPin {
			channel,
			unexport_on_drop: true,
		}
	}

	pub fn export(&mut self) -> io::Result<bool> {
		sysfs::export(self.channel)
	}

	pub fn unexport(&mut self) -> io::Result<()> {
		sysfs::unexport(self.channel)
	}

	pub fn set_enabled(&mut self, enabled: bool) -> io::Result<()> {
		sysfs::set_enabled(self.channel, enabled)
	}

	pub fn enabled(&self) -> io::Result<bool> {
		sysfs::enabled(self.channel)
	}

	pub fn set_pulse_width(&mut self, pulse_width_ns: u64) -> io::Result<()> {
		sysfs::set_pulse_width(self.channel, pulse_width_ns)
	}

	pub fn pulse_width(&self) -> io::Result<u64> {
		sysfs::pulse_width(self.channel)
	}

	pub fn set_period(&mut self, period_ns: u64) -> io::Result<()> {
		sysfs::set_period(self.channel, period_ns)
	}

	pub fn period(&self) -> io::Result<u64> {
		sysfs::period(self.channel)
	}

	pub fn set_polarity(&mut self, polarity: Polarity) -> io::Result<()> {
		sysfs::set_polarity(self.channel, polarity)
	}

	pub fn polarity(&self) -> io::Result<Polarity> {
		sysfs::polarity(self.channel)
	}
}

impl Drop for PwmPin {
	fn drop(&mut self) {
		if self.unexport_on_drop {
			self.unexport().unwrap_or_default()
		}
	}
}