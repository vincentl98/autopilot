#![no_std]

/// Adapted from https://sensorian.github.io/c_docs/_example1_2_m_p_l3115_a2_8c_source.html
/// This implementation is incomplete as it does not allow one-shot reading
/// Note: `pa` means `pressure_or_altitude`

use embedded_hal::blocking::i2c::{WriteRead, Write};

#[allow(dead_code)]
mod registers;

#[allow(dead_code)]
mod constants;

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum PowerMode {
	Standby,
	Active,
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum SensorMode {
	Pressure,
	Altitude,
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum OversamplingRatio {
	Ratio1,
	Ratio2,
	Ratio4,
	Ratio8,
	Ratio16,
	Ratio32,
	Ratio64,
	Ratio128,
}

#[derive(Debug)]
pub enum Error<E> {
	I2c(E),
	DataNotReady,
}

pub struct Mpl3115A2<I2C> {
	i2c: I2C,
	sensor_mode: SensorMode,
	power_mode: PowerMode,
}

// These values are set on reset
const RESET_SENSOR_MODE: SensorMode = SensorMode::Pressure;
const RESET_POWER_MODE: PowerMode = PowerMode::Standby;

impl<I2C, E> Mpl3115A2<I2C> where I2C: Write<Error=E> + WriteRead<Error=E> {
	pub fn new(i2c: I2C) -> Self {
		Self {
			i2c,
			sensor_mode: RESET_SENSOR_MODE,
			power_mode: RESET_POWER_MODE,
		}
	}
	pub fn init(&mut self,
				power_mode: PowerMode,
				sensor_mode: SensorMode,
				oversampling_ratio: OversamplingRatio) -> Result<(), Error<E>> {
		self.identify()?;

		self.set_standby_mode()?;

		self.write_u8(registers::PT_DATA_CFG,
					  constants::DREM | constants::PDEFE | constants::TDEFE)?;


		let oversampling_ratio = {
			match oversampling_ratio {
				OversamplingRatio::Ratio1 => constants::OS_1,
				OversamplingRatio::Ratio2 => constants::OS_2,
				OversamplingRatio::Ratio4 => constants::OS_4,
				OversamplingRatio::Ratio8 => constants::OS_8,
				OversamplingRatio::Ratio16 => constants::OS_16,
				OversamplingRatio::Ratio32 => constants::OS_32, // 130 ms
				OversamplingRatio::Ratio64 => constants::OS_64, // 258 ms
				OversamplingRatio::Ratio128 => constants::OS_128, // 512 ms
			}
		};

		self.write_u8(registers::CTRL_REG1,
					  oversampling_ratio | constants::ACTIVE)?;
		self.power_mode = PowerMode::Active; // Most likely

		self.set_power_mode(power_mode)?;
		self.set_sensor_mode(sensor_mode)?;

		Ok(())
	}

	#[inline(always)]
	pub fn power_mode(&mut self) -> Result<PowerMode, Error<E>> {
		let status = self.read_u8(registers::SYS_MOD)?;

		if status & constants::ACTIVE == 0 {
			Ok(PowerMode::Standby)
		} else {
			Ok(PowerMode::Active)
		}
	}

	#[inline(always)]
	pub fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<E>> {
		match mode {
			PowerMode::Active => self.set_active_mode(),
			PowerMode::Standby => self.set_standby_mode(),
		}
	}

	fn set_standby_mode(&mut self) -> Result<(), Error<E>> {
		if self.power_mode != PowerMode::Standby {
			let mut register = self.control_1()?;
			register &= !constants::ACTIVE; // Clear SBYB bit for Standby mode
			self.write_u8(registers::CTRL_REG1, register)?; // Put device in Standby mode
		}
		Ok(())
	}

	fn set_active_mode(&mut self) -> Result<(), Error<E>> {
		let mut settings = self.control_1()?;
		settings |= constants::ACTIVE; // Set SBYB bit for Active mode
		self.write_u8(registers::CTRL_REG1, settings)?;
		Ok(())
	}

	pub fn set_sensor_mode(&mut self, mode: SensorMode) -> Result<(), Error<E>> {
		match mode {
			SensorMode::Altitude => self.set_altitude_mode()?,
			SensorMode::Pressure => self.set_pressure_mode()?,
		}

		self.sensor_mode = mode;
		Ok(())
	}

	fn set_altitude_mode(&mut self) -> Result<(), Error<E>> {
		let mut register = self.control_1()?;
		register &= !constants::ACTIVE;
		self.write_u8(registers::CTRL_REG1, register)?;
		register |= constants::ALT | constants::ACTIVE;
		self.write_u8(registers::CTRL_REG1, register)?;
		Ok(())
	}

	#[inline(always)]
	pub fn sensor_mode(&mut self) -> Result<SensorMode, Error<E>> {
		if self.control_1()? & constants::ALT != 0 {
			Ok(SensorMode::Altitude)
		} else {
			Ok(SensorMode::Pressure)
		}
	}

	fn set_pressure_mode(&mut self) -> Result<(), Error<E>> {
		let mut register = self.control_1()?;
		register &= !constants::ACTIVE;
		self.write_u8(registers::CTRL_REG1, register)?;
		register &= !constants::ALT;
		register |= constants::ACTIVE;
		self.write_u8(registers::CTRL_REG1, register)?;
		Ok(())
	}

	pub fn read_pa(&mut self) -> Result<f32, Error<E>> {
		let mut buffer = [0u8; 4];
		self.i2c
			.write_read(constants::ADDRESS, &[registers::STATUS], &mut buffer)
			.map_err(|e| Error::I2c(e))?;

		let status = buffer[0];
		let p_msb = buffer[1];
		let p_csb = buffer[2];
		let p_lsb = buffer[3];

		const ONE_SIXTEENTH: f32 = 1. / 16.;
		const ONE_QUARTER: f32 = 1. / 4.;

		if self.is_pa_data_available(status) {
			if self.sensor_mode == SensorMode::Altitude {
				let integer = i16::from_be_bytes([p_msb, p_csb]);
				let fractional = u8::from_be_bytes([p_lsb >> 4]);
				let altitude = integer as f32 + fractional as f32 * ONE_SIXTEENTH;

				Ok(altitude)
			} else {
				let integer = u32::from_be_bytes([0u8, p_msb >> 6, p_msb << 2 | p_csb >> 6,
					p_csb << 2 | p_lsb >> 6]);
				let fractional = u8::from_be_bytes([(p_lsb << 2) >> 6]);
				let pressure = integer as f32 + fractional as f32 * ONE_QUARTER;

				Ok(pressure)
			}
		} else {
			Err(Error::DataNotReady)
		}
	}

	pub fn read_pa_temperature(&mut self) -> Result<(f32, f32), Error<E>> {  // (PA, T)
		let mut buffer = [0u8; 6];
		self.i2c
			.write_read(constants::ADDRESS, &[registers::STATUS], &mut buffer)
			.map_err(|e| Error::I2c(e))?;

		let status = buffer[0];
		let p_msb = buffer[1];
		let p_csb = buffer[2];
		let p_lsb = buffer[3];
		let t_msb = buffer[4];
		let t_lsb = buffer[5];

		const ONE_SIXTEENTH: f32 = 1. / 16.;
		const ONE_QUARTER: f32 = 1. / 4.;

		if self.is_temperature_data_available(status) 
			&& self.is_pa_data_available(status) {
			
			let integer = i8::from_be_bytes([t_msb]);
			let fractional = u8::from_be_bytes([t_lsb >> 4]);

			let temperature = integer as f32 + fractional as f32 * ONE_SIXTEENTH;

			let pa = {
				if self.sensor_mode == SensorMode::Altitude {
					let integer = i16::from_be_bytes([p_msb, p_csb]);
					let fractional = u8::from_be_bytes([p_lsb >> 4]);
					let altitude = integer as f32 + fractional as f32 * ONE_SIXTEENTH;

					altitude
				} else {
					let integer = u32::from_be_bytes([0u8, p_msb >> 6, p_msb << 2 | p_csb >> 6,
						p_csb << 2 | p_lsb >> 6]);
					let fractional = u8::from_be_bytes([(p_lsb << 2) >> 6]);
					let pressure = integer as f32 + fractional as f32 / ONE_QUARTER;

					pressure
				}
			};

			Ok((pa, temperature))
		} else {
			Err(Error::DataNotReady)
		}
	}

	#[inline(always)]
	pub fn set_altitude_offset(&mut self, offset: i8) -> Result<(), Error<E>> {
		self.write_u8(registers::OFF_H, offset.to_be_bytes()[0])
	}

	#[inline(always)]
	pub fn identify(&mut self) -> Result<bool, Error<E>> {
		Ok(self.read_u8(registers::WHO_AM_I)? == constants::CHIP_ID)
	}

	#[inline(always)]
	fn status(&mut self) -> Result<u8, Error<E>> {
		Ok(self.read_u8(registers::STATUS)?)
	}

	#[inline(always)]
	fn control_1(&mut self) -> Result<u8, Error<E>> {
		Ok(self.read_u8(registers::CTRL_REG1)?)
	}

	#[inline(always)]
	fn is_temperature_data_available(&self, status: u8) -> bool {
		status & 0b10 != 0
	}

	#[inline(always)]
	fn is_pa_data_available(&self, status: u8) -> bool {
		status & 0b100 != 0
	}

	#[inline(always)]
	fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), Error<E>> {
		self.i2c.write(constants::ADDRESS, &[reg, value])
			.map_err(|e| Error::I2c(e))?;

		Ok(())
	}

	fn read_u8(&mut self, register: u8) -> Result<u8, Error<E>> {
		let mut buffer = [0u8; 1];
		self.i2c.write_read(constants::ADDRESS, &[register], &mut buffer)
			.map_err(|e| Error::I2c(e))?;

		Ok(buffer[0])
	}
}

#[cfg(test)]
mod tests {
	// TODO
}