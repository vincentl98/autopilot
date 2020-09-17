use crate::output::{OutputController};
use rppal::gpio::{Level, OutputPin};
use rppal::pwm::{Pwm, Polarity, Channel};

pub struct MotorPins {
	pub pwm_channel: Channel,
	pub pin_in_1: u8,
	pub pin_in_2: u8,
}

#[derive(Debug)]
struct Motor {
	pub pwm: Pwm,
	pub in_1: OutputPin,
	pub in_2: OutputPin,
}

impl Motor {
	pub fn new(pins: MotorPins) -> Self {
		let gpio = rppal::gpio::Gpio::new().expect("Failed to open GPIO");
		Motor {
			in_1: gpio.get(pins.pin_in_1).unwrap().into_output(),
			in_2: gpio.get(pins.pin_in_2).unwrap().into_output(),
			pwm: rppal::pwm::Pwm::with_frequency(pins.pwm_channel,
												 50.0,
												 0.,
												 Polarity::Normal,
												 false)
				.expect(format!("Failed to open PWM channel {:?}", pins.pwm_channel)
					.as_str()),
		}
	}

	pub fn set_power(&mut self, power: f64) -> anyhow::Result<()> {
		self.set_duty_cycle(power.abs())?;

		if power > 0. {
			self.backward();
		} else {
			self.forward();
		}

		Ok(())
	}

	pub fn set_duty_cycle(&mut self, duty_cycle: f64) -> anyhow::Result<()> {
		Ok(self.pwm.set_duty_cycle(duty_cycle)?)
	}

	pub fn forward(&mut self) {
		self.in_1.write(Level::High);
		self.in_2.write(Level::Low);
	}

	pub fn backward(&mut self) {
		self.in_1.write(Level::Low);
		self.in_2.write(Level::High);
	}

	pub fn brake(&mut self) {
		self.in_1.write(Level::High);
		self.in_2.write(Level::High);
	}

	pub fn idle(&mut self) {
		self.in_1.write(Level::Low);
		self.in_2.write(Level::Low);
	}
}

#[derive(Debug)]
pub struct L298NOutputController {
	left_motor: Motor,
	right_motor: Motor,
}

impl L298NOutputController {
	pub fn new(left_motor_pins: MotorPins, right_motor_pins: MotorPins) -> anyhow::Result<Self> {
		let mut instance = L298NOutputController {
			left_motor: Motor::new(left_motor_pins),
			right_motor: Motor::new(right_motor_pins),
		};

		instance.right_motor.pwm.enable()?;
		instance.right_motor.idle();
		instance.right_motor.set_duty_cycle(0.)?;

		instance.left_motor.pwm.enable()?;
		instance.left_motor.idle();
		instance.left_motor.set_duty_cycle(0.)?;

		Ok(instance)
	}
}

impl OutputController<(f64, f64)> for L298NOutputController {
	fn write_output(&mut self, output: (f64, f64)) -> anyhow::Result<()> {
		let (left_power, right_power) = output;

		const THRESHOLD: f64 = 0.05;

		if right_power.abs() > THRESHOLD {
			self.right_motor.set_power(right_power)?;
		} else {
			self.right_motor.brake();
		}

		if left_power.abs() > THRESHOLD {
			self.left_motor.set_power(left_power)?;
		} else {
			self.left_motor.brake();
		}

		Ok(())
	}
}