#[no_std]
/// Adapted from https://sensorian.github.io/c_docs/_example1_2_m_p_l3115_a2_8c_source.html
use embedded_hal::blocking::i2c::{WriteRead, Write};
use std::time::Duration;
use std::marker::PhantomData;

#[allow(dead_code)]
mod registers;

#[allow(dead_code)]
mod constants;

#[derive(Eq, PartialEq)]
pub enum PowerMode {
	Standby,
	Active,
}

#[derive(Eq, PartialEq)]
pub enum SensorMode {
	Pressure,
	Altitude,
}

pub enum OversamplingRatio {
	Ratio1,
	Ratio2,
	Ratio16,
	Ratio32,
	Ratio64,
	Ratio128,
}

pub struct Mpl3115a2<I2C> {
	i2c: PhantomData<I2C>,
	sensor_mode: SensorMode,
}

// These values are set on reset
const DEFAULT_SENSOR_MODE: SensorMode = SensorMode::Pressure;
const DEFAULT_POWER_MODE: PowerMode = PowerMode::Standby;

impl<I2C, E> Mpl3115a2<I2C>
	where I2C: Write<Error=E> + WriteRead<Error=E> {
	pub fn new(i2c: I2C) -> Self {
		Self {
			i2c,
			sensor_mode: DEFAULT_SENSOR_MODE, // Will be overriden by call to `init`
		}
	}

	pub fn init(&mut self,
				power_mode: PowerMode,
				sensor_mode: SensorMode,
				oversampling_ratio: OversamplingRatio) -> Result<(), E> {

        self.identify()?;

		instance.set_standby_mode()?;
		instance.write_register(registers::PT_DATA_CFG,
								constants::DREM | constants::PDEFE | constants::TDEFE)?;


		let oversampling_ratio = {
			match oversampling_ratio {
				OversamplingRatio::Ratio1 => constants::OS_1,
				OversamplingRatio::Ratio2 => constants::OS_2,
				OversamplingRatio::Ratio16 => constants::OS_16,
				OversamplingRatio::Ratio32 => constants::OS_32, // 130 ms
				OversamplingRatio::Ratio64 => constants::OS_64, // 258 ms
				OversamplingRatio::Ratio128 => constants::OS_128, // 512 ms
			}
		};


		instance.write_register(registers::CTRL_REG1,
								oversampling_ratio | constants::ACTIVE)?;

		if power_mode != DEFAULT_POWER_MODE {
			instance.set_active_mode()?;
		}

        self.sensor_mode = sensor_mode;

		if instance.sensor_mode == DEFAULT_SENSOR_MODE {
			instance.set_altitude_mode()?;
		}

		Ok(instance)
	}

	fn write_register(&mut self, register: u8, value: u8) -> anyhow::Result<()> {
		let mut i2c = self.i2c_try_lock()?;
		i2c.set_slave_address(constants::I2C_ADDRESS)?;
		i2c.write(&[register, value])?;
		Ok(())
	}

	fn read_register(&mut self, register: u8) -> anyhow::Result<u8> {
        self.i2c.write()
		let mut i2c = self.i2c_try_lock()?;
		i2c.set_slave_address(constants::I2C_ADDRESS)?;

		let mut buffer = [0u8; 1];
		i2c.write_read(&[register], &mut buffer)?;

		Ok(buffer[0])
	}

	pub fn power_mode(&mut self) -> anyhow::Result<PowerMode> {
		let status = self.read_register(registers::SYS_MOD)?;

		if status & constants::ACTIVE == 0 {
			Ok(PowerMode::Standby)
		} else {
			Ok(PowerMode::Active)
		}
	}

	pub fn set_power_mode(&mut self, mode: PowerMode) -> anyhow::Result<()> {
		match mode {
			PowerMode::Active => self.set_active_mode(),
			PowerMode::Standby => self.set_standby_mode(),
		}
	}

	fn set_standby_mode(&mut self) -> anyhow::Result<()> {
		let mut register = self.control_1()?;
		register &= !constants::ACTIVE; // Clear SBYB bit for Standby mode
		self.write_register(registers::CTRL_REG1, register)?; // Put device in Standby mode
		Ok(())
	}

	fn set_active_mode(&mut self) -> anyhow::Result<()> {
		let mut settings = self.control_1()?;
		settings |= constants::ACTIVE; // Set SBYB bit for Active mode
		self.write_register(registers::CTRL_REG1, settings)?;
		Ok(())
	}

	pub fn set_sensor_mode(&mut self, mode: SensorMode) -> anyhow::Result<()> {
		match mode {
			SensorMode::Altitude => self.set_altitude_mode()?,
			SensorMode::Pressure => self.set_pressure_mode()?,
		}

		self.sensor_mode = mode;
		Ok(())
	}

	fn set_altitude_mode(&mut self) -> anyhow::Result<()> {
		let mut register = self.control_1()?;
		register &= !constants::ACTIVE;
		self.write_register(registers::CTRL_REG1, register)?;
		register |= constants::ALT | constants::ACTIVE;
		self.write_register(registers::CTRL_REG1, register)?;
		Ok(())
	}

	pub fn sensor_mode(&mut self) -> anyhow::Result<SensorMode> {
		if self.control_1()? & constants::ALT != 0 {
			Ok(SensorMode::Altitude)
		} else {
			Ok(SensorMode::Pressure)
		}
	}

	fn set_pressure_mode(&mut self) -> anyhow::Result<()> {
		let mut register = self.control_1()?;
		register &= !constants::ACTIVE;
		self.write_register(registers::CTRL_REG1, register)?;
		register &= !constants::ALT;
		register |= constants::ACTIVE;
		self.write_register(registers::CTRL_REG1, register)?;
		Ok(())
	}

	pub fn read(&mut self) -> anyhow::Result<(Option<f64>, Option<f64>)> {  // (T, P)
		let mut buffer = [0u8; 6];
		self.i2c_try_lock()?.write_read(&[registers::STATUS], &mut buffer)?;

		let status = buffer[0];
		let p_msb = buffer[1];
		let p_csb = buffer[2];
		let p_lsb = buffer[3];
		let t_msb = buffer[4];
		let t_lsb = buffer[5];

		let mut temperature = None;

		if Mpl3115a2::is_temperature_data_available(status) {
			let integer = i8::from_be_bytes([t_msb]);
			let fractional = u8::from_be_bytes([t_lsb >> 4]);

			temperature = Some(integer as f64 + fractional as f64 / 16.)
		}

		if Mpl3115a2::is_pressure_or_altitude_data_available(status) {
			if self.sensor_mode == SensorMode::Altitude {
				let integer = i16::from_be_bytes([p_msb, p_csb]);
				let fractional = u8::from_be_bytes([p_lsb >> 4]);
				let altitude = integer as f64 + fractional as f64 / 16.;

				Ok((temperature,
					Some(altitude)))
			} else {
				let integer = u32::from_be_bytes([0u8, p_msb >> 6, p_msb << 2 | p_csb >> 6,
					p_csb << 2 | p_lsb >> 6]);
				let fractional = u8::from_be_bytes([(p_lsb << 2) >> 6]);
				let pressure = integer as f64 + fractional as f64 / 4.;

				Ok((temperature,
					Some(pressure)))
			}
		} else {
			Ok((temperature, None))
		}
	}

	pub fn identify(&mut self) -> anyhow::Result<()> {
		if self.read_register(registers::WHO_AM_I)? == constants::CHIP_ID {
			Ok(())
		} else {
			Err(anyhow!("Chip ID not recognized"))
		}
	}

	fn status(&mut self) -> anyhow::Result<u8> {
		Ok(self.read_register(registers::STATUS)?)
	}

	fn control_1(&mut self) -> anyhow::Result<u8> {
		Ok(self.read_register(registers::CTRL_REG1)?)
	}
	fn is_temperature_data_available(status: u8) -> bool {
		status & 0b10 != 0
	}

	fn is_pressure_or_altitude_data_available(status: u8) -> bool {
		status & 0b100 != 0
	}
}

#[cfg(test)]
mod tests {
	/// TODO
}
