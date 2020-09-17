use std::marker::PhantomData;
use embedded_hal::blocking::i2c::{WriteRead, Write};

/// Note: use existing library
pub const ADDRESS: u8 = 0x29;

pub struct Bno055<I2C> {
	i2c: PhantomData<I2C>,
	address: u8,
}

impl<I2C, E> Bno055<I2C>
	where I2C: WriteRead<Error=E> + Write<Error=E> {
	pub fn new(address: u8) -> Self {
		Bno055 {
			i2c: PhantomData,
			address,
		}
	}

	pub fn set_ndof_mode(&self, i2c: &mut I2C) -> Result<(), E> {
		const OPR_MODE_REGISTER: u8 = 0x3D;
		const NDOF: u8 = 0b00001100;

		i2c.try_write(self.address, &[OPR_MODE_REGISTER, NDOF])
	}

	pub fn set_unit_deg(&self, i2c: &mut I2C) -> Result<(), E> {
		const UNIT_SEL_REGISTER: u8 = 0x3B;
		const DEG_MASK: u8 = 0b11111011;

		let mut buffer = [0u8; 1];
		i2c.try_write_read(self.address, &[UNIT_SEL_REGISTER], &mut buffer)?;
		i2c.try_write(self.address, &[UNIT_SEL_REGISTER, buffer[0] & DEG_MASK])
	}

	pub fn identify(&self, i2c: &mut I2C) -> Result<bool, E> {
		const CHIP_ID_REGISTER: u8 = 0x0;
		const CHIP_ID: u8 = 0xA0;

		let mut buffer = [0u8; 1];

		i2c.try_write_read(self.address, &[CHIP_ID_REGISTER], &mut buffer)?;

		Ok(buffer[0] == CHIP_ID)
	}

	pub(crate) fn orientation_euler(&self, i2c: &mut I2C) -> Result<(f32, f32, f32), E> {
		const EUL_HEADING_LSB_REGISTER: u8 = 0x1A;

		let mut buffer = [0u8; 6];

		i2c.try_write_read(self.address, &[EUL_HEADING_LSB_REGISTER], &mut buffer)?;

		const LSB_TO_DEG: f32 = 1. / 16.;

		let lsb_heading = i16::from_le_bytes([buffer[0], buffer[1]]);
		let heading = (lsb_heading as f32 * LSB_TO_DEG)
			.max(0.)
			.min(360.);

		let lsb_roll = i16::from_le_bytes([buffer[2], buffer[3]]);
		let roll = (lsb_roll as f32 * LSB_TO_DEG)
			.max(0.)
			.min(360.);

		let lsb_pitch = i16::from_le_bytes([buffer[4], buffer[5]]);
		let pitch = (lsb_pitch as f32 * LSB_TO_DEG)
			.max(0.)
			.min(360.);

		Ok((heading, roll, pitch))
	}
}