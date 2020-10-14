use spidev::{Spidev, SpidevOptions, SpiModeFlags, SpidevTransfer};
use std::io::ErrorKind;
use std::{thread, io};
use std::time::{Duration, Instant};
use nalgebra::Vector3;

use autopilot::{ImuData, G};

use constants::*;
use registers::*;
use std::f32::consts::PI;

/// LSM9DS1 driver using Linux `Spidev`
pub struct LSM9DS1 {
	acc_gyr: Spidev,
	mag: Spidev,
}


pub const OUTPUT_DELAY: Duration = Duration::from_micros(4202);

impl LSM9DS1 {
	pub fn new(acc_gyr_spi_path: &str, mag_spi_path: &str) -> Result<Self, io::Error> {
		let options = SpidevOptions::new()
			.bits_per_word(8)
			.max_speed_hz(10_000_000) // datasheet p. 15
			.mode(SpiModeFlags::SPI_MODE_0)
			.build();

		let mut acc_gyr = Spidev::open(acc_gyr_spi_path)?;
		acc_gyr.configure(&options)?;

		let mut mag = Spidev::open(mag_spi_path)?;
		mag.configure(&options)?;

		Ok(Self {
			acc_gyr,
			mag,
		})
	}

	fn identify(&mut self) -> Result<(), io::Error> {
		const ACC_GYR_CHIP_ID: u8 = 0b01101000;

		if self.acc_gyr.read_register(registers::WHO_AM_I)? != ACC_GYR_CHIP_ID {
			return Err(io::Error::new(ErrorKind::AddrNotAvailable,
									  "Failed to identify accelerometer/gyroscope"));
		}

		const MAG_CHIP_ID: u8 = 0b00111101;
		if self.mag.read_register(registers::WHO_AM_I)? != MAG_CHIP_ID {
			return Err(io::Error::new(ErrorKind::AddrNotAvailable,
									  "Failed to identify magnetometer"));
		}

		Ok(())
	}

	pub fn init(mut self) -> Result<Self, io::Error> {
		self.identify()?;

		self.acc_gyr.write_register(CTRL_REG8, SW_RESET)?;

		const US_200: Duration = Duration::from_millis(200);
		thread::sleep(US_200);

		self.acc_gyr.write_register(CTRL_REG1_G, ODR_GYR_952_HZ | FS_GYR_500_DPS)?;

		const GYR_ENABLE_3_AXIS: u8 = XEN_G | YEN_G | ZEN_G;
		self.acc_gyr.write_register(CTRL_REG4, GYR_ENABLE_3_AXIS)?;
		thread::sleep(US_200);

		const ACC_ENABLE_3_AXIS: u8 = XEN_ACC | YEN_ACC | ZEN_ACC;
		self.acc_gyr.write_register(CTRL_REG5_ACC, ACC_ENABLE_3_AXIS)?;
		self.acc_gyr.write_register(CTRL_REG6_ACC, ODR_ACC_952_HZ | FS_ACC_4G)?;

		thread::sleep(US_200);

		self.mag.write_register(CTRL_REG1_M, TEMP_COMP | OM_ULTRA_HIGH | ODR_M_80_HZ)?;
		self.mag.write_register(CTRL_REG2_M, FS_M_4G)?;

		self.mag.write_register(CTRL_REG3_M, MD_CONTINUOUS)?;
		self.mag.write_register(CTRL_REG4_M, OMZ_ULTRA_HIGH)?;
		self.mag.write_register(CTRL_REG5_M, 0)?;

		thread::sleep(US_200);

		Ok(self)
	}

	pub fn read_output(&mut self) -> Result<([Vector3<f32>; 3], Instant), io::Error> {
		const ACC_4G_SCALE: f32 = 4. * G / i16::max_value() as f32;
		const GYR_500_DPS_SCALE: f32 = 500. * PI / i16::max_value() as f32 / 180.;
		const MAG_4G_SCALE: f32 = 4. / i16::max_value() as f32;

		let instant = Instant::now();

		let acc_output = self.read_acc_or_gyr_output(OUT_X_L_ACC)?;

		let acc_x = i16::from_le_bytes([acc_output[0], acc_output[1]]) as f32 * ACC_4G_SCALE;
		let acc_y = i16::from_le_bytes([acc_output[2], acc_output[3]]) as f32 * ACC_4G_SCALE;
		let acc_z = i16::from_le_bytes([acc_output[4], acc_output[5]]) as f32 * ACC_4G_SCALE;

		let gyr_output = self.read_acc_or_gyr_output(OUT_X_L_G)?;

		let gyr_x = i16::from_le_bytes([gyr_output[0], gyr_output[1]]) as f32 * GYR_500_DPS_SCALE;
		let gyr_y = i16::from_le_bytes([gyr_output[2], gyr_output[3]]) as f32 * GYR_500_DPS_SCALE;
		let gyr_z = i16::from_le_bytes([gyr_output[4], gyr_output[5]]) as f32 * GYR_500_DPS_SCALE;

		let mag_output = self.read_mag_output(OUT_X_L_M)?;

		let mag_x = i16::from_le_bytes([mag_output[0], mag_output[1]]) as f32 * MAG_4G_SCALE;
		let mag_y = i16::from_le_bytes([mag_output[2], mag_output[3]]) as f32 * MAG_4G_SCALE;
		let mag_z = i16::from_le_bytes([mag_output[4], mag_output[5]]) as f32 * MAG_4G_SCALE;

		let acc = Vector3::<f32>::new(acc_x, acc_y, acc_z);
		let gyr = Vector3::<f32>::new(gyr_x, gyr_y, gyr_z);
		let mag = Vector3::<f32>::new(mag_x, mag_y, mag_z);

		Ok(([acc, gyr, mag], instant))
	}

	#[inline(always)]
	fn read_acc_or_gyr_output(&mut self, register: u8) -> Result<[u8; 6], io::Error> {
		let mut rx_buffer = [0u8; 7];
		let mut tx_buffer = [0u8; 7];
		tx_buffer[0] = register | READ_FLAG;

		self.acc_gyr.transfer(&mut SpidevTransfer::read_write(&tx_buffer, &mut rx_buffer))?;

		Ok([rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4], rx_buffer[5], rx_buffer[6]])
	}

	#[inline(always)]
	fn read_mag_output(&mut self, register: u8) -> Result<[u8; 6], io::Error> {
		let mut rx_buffer = [0u8; 7];
		let mut tx_buffer = [0u8; 7];
		tx_buffer[0] = register | ACC_GYR_MULTIPLE_READ_FLAG;

		self.mag.transfer(&mut SpidevTransfer::read_write(&tx_buffer, &mut rx_buffer))?;

		Ok([rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4], rx_buffer[5], rx_buffer[6]])
	}
}

trait SpiRegisterInterface {
	fn write_register(&mut self, register: u8, data: u8) -> Result<u8, io::Error>;
	fn read_register(&mut self, register: u8) -> Result<u8, io::Error>;
}

impl SpiRegisterInterface for Spidev {
	#[inline(always)]
	fn write_register(&mut self, register: u8, data: u8) -> Result<u8, io::Error> {
		let mut rx_buffer = [0u8; 2];

		self.transfer(&mut SpidevTransfer::read_write(&[register, data], &mut rx_buffer))?;

		Ok(rx_buffer[1])
	}

	#[inline(always)]
	fn read_register(&mut self, register: u8) -> Result<u8, io::Error> {
		self.write_register(register | READ_FLAG, 0u8)
	}
}

#[allow(dead_code)]
mod registers {
	// Accelerometer and Gyroscope registers
	pub const ACT_THS: u8 = 0x04;
	pub const ACT_DUR: u8 = 0x05;
	pub const INT_GEN_CFG_ACC: u8 = 0x06;
	pub const INT_GEN_THS_X_ACC: u8 = 0x07;
	pub const INT_GEN_THS_Y_ACC: u8 = 0x08;
	pub const INT_GEN_THS_Z_ACC: u8 = 0x09;
	pub const INT_GEN_DUR_ACC: u8 = 0x0A;
	pub const REFERENCE_G: u8 = 0x0B;
	pub const INT1_CTRL: u8 = 0x0C;
	pub const INT2_CTRL: u8 = 0x0D;
	pub const WHO_AM_I: u8 = 0x0F;
	pub const CTRL_REG1_G: u8 = 0x10;
	pub const CTRL_REG2_G: u8 = 0x11;
	pub const CTRL_REG3_G: u8 = 0x12;
	pub const ORIENT_CFG_G: u8 = 0x13;
	pub const INT_GEN_SRC_G: u8 = 0x14;
	pub const OUT_TEMP_L: u8 = 0x15;
	pub const OUT_TEMP_H: u8 = 0x16;
	pub const STATUS_REG: u8 = 0x17;
	pub const OUT_X_L_G: u8 = 0x18;
	pub const OUT_X_H_G: u8 = 0x19;
	pub const OUT_Y_L_G: u8 = 0x1A;
	pub const OUT_Y_H_G: u8 = 0x1B;
	pub const OUT_Z_L_G: u8 = 0x1C;
	pub const OUT_Z_H_G: u8 = 0x1D;
	pub const CTRL_REG4: u8 = 0x1E;
	pub const CTRL_REG5_ACC: u8 = 0x1F;
	pub const CTRL_REG6_ACC: u8 = 0x20;
	pub const CTRL_REG7_ACC: u8 = 0x21;
	pub const CTRL_REG8: u8 = 0x22;
	pub const CTRL_REG9: u8 = 0x23;
	pub const CTRL_REG10: u8 = 0x24;
	pub const INT_GEN_SRC_ACC: u8 = 0x26;
	pub const OUT_X_L_ACC: u8 = 0x28;
	pub const OUT_X_H_ACC: u8 = 0x29;
	pub const OUT_Y_L_ACC: u8 = 0x2A;
	pub const OUT_Y_H_ACC: u8 = 0x2B;
	pub const OUT_Z_L_ACC: u8 = 0x2C;
	pub const OUT_Z_H_ACC: u8 = 0x2D;
	pub const FIFO_CTRL: u8 = 0x2E;
	pub const FIFO_SRC: u8 = 0x2F;
	pub const INT_GEN_CFG_G: u8 = 0x30;
	pub const INT_GEN_THS_XH_G: u8 = 0x31;
	pub const INT_GEN_THS_ACC_G: u8 = 0x32;
	pub const INT_GEN_THS_YH_G: u8 = 0x33;
	pub const INT_GEN_THS_YL_G: u8 = 0x34;
	pub const INT_GEN_THS_ZH_G: u8 = 0x35;
	pub const INT_GEN_THS_ZL_G: u8 = 0x36;
	pub const INT_GEN_DUR_G: u8 = 0x37;

	// Magnetometer registers
	pub const OFFSET_X_REG_L_M: u8 = 0x05;
	pub const OFFSET_X_REG_H_M: u8 = 0x06;
	pub const OFFSET_Y_REG_L_M: u8 = 0x07;
	pub const OFFSET_Y_REG_H_M: u8 = 0x08;
	pub const OFFSET_Z_REG_L_M: u8 = 0x09;
	pub const OFFSET_Z_REG_H_M: u8 = 0x0A;
	pub const CTRL_REG1_M: u8 = 0x20;
	pub const CTRL_REG2_M: u8 = 0x21;
	pub const CTRL_REG3_M: u8 = 0x22;
	pub const CTRL_REG4_M: u8 = 0x23;
	pub const CTRL_REG5_M: u8 = 0x24;
	pub const STATUS_REG_M: u8 = 0x27;
	pub const OUT_X_L_M: u8 = 0x28;
	pub const OUT_X_H_M: u8 = 0x29;
	pub const OUT_Y_L_M: u8 = 0x2A;
	pub const OUT_Y_H_M: u8 = 0x2B;
	pub const OUT_Z_L_M: u8 = 0x2C;
	pub const OUT_Z_H_M: u8 = 0x2D;
	pub const INT_CFG_M: u8 = 0x30;
	pub const INT_SRC_M: u8 = 0x31;
	pub const INT_THS_L_M: u8 = 0x32;
	pub const INT_THS_H_M: u8 = 0x33;
}

#[allow(dead_code)]
mod constants {
	pub const READ_FLAG: u8 = 0x80;
	pub const ACC_GYR_MULTIPLE_READ_FLAG: u8 = READ_FLAG | 0x40;

	pub const SW_RESET: u8 = 1;

	// Configuration bits Accelerometer and Gyroscope
	pub const XEN_G: u8 = 0x08;
	pub const YEN_G: u8 = 0x10;
	pub const ZEN_G: u8 = 0x20;
	pub const XEN_ACC: u8 = 0x08;
	pub const YEN_ACC: u8 = 0x10;
	pub const ZEN_ACC: u8 = 0x20;
	pub const ODR_GYR_14_900_MHZ: u8 = 0x20;
	pub const ODR_GYR_59_500_MHZ: u8 = 0x40;
	pub const ODR_GYR_119_HZ: u8 = 0x60;
	pub const ODR_GYR_238_HZ: u8 = 0x80;
	pub const ODR_GYR_476_HZ: u8 = 0xA0;
	pub const ODR_GYR_952_HZ: u8 = 0xC0;
	pub const ODR_ACC_10_HZ: u8 = 0x20;
	pub const ODR_ACC_50_HZ: u8 = 0x40;
	pub const ODR_ACC_119_HZ: u8 = 0x60;
	pub const ODR_ACC_238_HZ: u8 = 0x80;
	pub const ODR_ACC_476_HZ: u8 = 0xA0;
	pub const ODR_ACC_952_HZ: u8 = 0xC0;
	pub const FS_GYR_MASK: u8 = 0xE3;
	pub const FS_GYR_245_DPS: u8 = 0x00;
	pub const FS_GYR_500_DPS: u8 = 0x08;
	pub const FS_GYR_2000_DPS: u8 = 0x18;
	pub const FS_ACC_MASK: u8 = 0xE7;
	pub const FS_ACC_2G: u8 = 0x00;
	pub const FS_ACC_4G: u8 = 0x10;
	pub const FS_ACC_8G: u8 = 0x18;
	pub const FS_ACC_16G: u8 = 0x08;

	// Configuration bits Magnetometer
	pub const TEMP_COMP: u8 = 0x80;
	pub const OM_LOW: u8 = 0x00;
	pub const OM_MEDIUM: u8 = 0x20;
	pub const OM_HIGH: u8 = 0x40;
	pub const OM_ULTRA_HIGH: u8 = 0x60;
	pub const ODR_M_625_MHZ: u8 = 0x00;
	pub const ODR_M_1250_MHZ: u8 = 0x04;
	pub const ODR_M_25_MHZ: u8 = 0x08;
	pub const ODR_M_5_HZ: u8 = 0x0C;
	pub const ODR_M_10_HZ: u8 = 0x10;
	pub const ODR_M_20_HZ: u8 = 0x14;
	pub const ODR_M_40_HZ: u8 = 0x18;
	pub const ODR_M_80_HZ: u8 = 0x1C;
	pub const FS_M_MASK: u8 = 0x0C;
	pub const FS_M_4G: u8 = 0x00;
	pub const FS_M_8G: u8 = 0x20;
	pub const FS_M_12G: u8 = 0x40;
	pub const FS_M_16G: u8 = 0x60;
	pub const MD_CONTINUOUS: u8 = 0x00;
	pub const MD_SINGLE: u8 = 0x01;
	pub const MD_POWER_DOWN: u8 = 0x02;
	pub const OMZ_LOW: u8 = 0x00;
	pub const OMZ_MEDIUM: u8 = 0x04;
	pub const OMZ_HIGH: u8 = 0x08;
	pub const OMZ_ULTRA_HIGH: u8 = 0x0C;
}