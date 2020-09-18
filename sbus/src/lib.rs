#![feature(array_map)]
/// Adapted from `sbus` library by Ze'ev Klapow

use arraydeque::{ArrayDeque, Wrapping};

pub const PACKET_SIZE: usize = 25;
pub const CHANNELS: usize = 16;


#[derive(Debug, Ord, PartialOrd, Eq, PartialEq, Copy, Clone)]
pub struct SbusPacket {
	pub channels: [u16; CHANNELS],
	pub digital_channels: [bool; 2],
	pub failsafe: bool,
	pub frame_lost: bool,
}

impl SbusPacket {
	pub fn normalized_channels(self) -> [f32; CHANNELS] {
		self.channels.map(Self::normalized_channel)
	}

	fn normalized_channel(channel: u16) -> f32 {
		const MIN: u16 = 172;
		const MAX: u16 = 1811;
		const RANGE: f32 = (MAX - MIN) as f32;

		-1.0 + (channel.min(MAX).max(MIN) - MIN) as f32 * 2.0 / RANGE
	}
}

pub struct SbusPacketParser {
	buffer: ArrayDeque<[u8; 50], Wrapping>,
}

impl SbusPacketParser {
	pub fn new() -> SbusPacketParser {
		SbusPacketParser {
			buffer: ArrayDeque::new(),
		}
	}

	pub fn push_bytes(&mut self, bytes: &[u8]) {
		bytes.iter().for_each(|b| {
			self.buffer.push_back(*b);
		})
	}

	pub fn try_parse(&mut self) -> Option<SbusPacket> {
		// The flag by should start with four zeros
		const FLAG_MASK: u8 = 0b11110000;
		const HEADER: u8 = 0x0F;
		const FOOTER: u8 = 0b00000000;

		if self.buffer.len() < PACKET_SIZE {
			None
		} else {
			if *self.buffer.get(PACKET_SIZE - 1).unwrap() == FOOTER
				&& self.buffer.get(PACKET_SIZE - 2).unwrap() & FLAG_MASK == 0
			{
				let mut data_bytes: [u16; 23] = [0; 23];
				for i in 0..23 {
					data_bytes[i] = self.buffer.pop_front().unwrap_or(0) as u16;
				}

				let mut channels: [u16; 16] = [0; 16];

				channels[0] = (((data_bytes[1]) | (data_bytes[2] << 8)) as u16 & 0x07FF).into();
				channels[1] = ((((data_bytes[2] >> 3) | (data_bytes[3] << 5)) as u16) & 0x07FF).into();
				channels[2] = ((((data_bytes[3] >> 6) | (data_bytes[4] << 2) | (data_bytes[5] << 10))
					as u16)
					& 0x07FF)
					.into();
				channels[3] = ((((data_bytes[5] >> 1) | (data_bytes[6] << 7)) as u16) & 0x07FF).into();
				channels[4] = ((((data_bytes[6] >> 4) | (data_bytes[7] << 4)) as u16) & 0x07FF).into();
				channels[5] = ((((data_bytes[7] >> 7) | (data_bytes[8] << 1) | (data_bytes[9] << 9))
					as u16)
					& 0x07FF)
					.into();
				channels[6] = ((((data_bytes[9] >> 2) | (data_bytes[10] << 6)) as u16) & 0x07FF).into();
				channels[7] =
					((((data_bytes[10] >> 5) | (data_bytes[11] << 3)) as u16) & 0x07FF).into();
				channels[8] = ((((data_bytes[12]) | (data_bytes[13] << 8)) as u16) & 0x07FF).into();
				channels[9] =
					((((data_bytes[13] >> 3) | (data_bytes[14] << 5)) as u16) & 0x07FF).into();
				channels[10] =
					((((data_bytes[14] >> 6) | (data_bytes[15] << 2) | (data_bytes[16] << 10)) as u16)
						& 0x07FF)
						.into();
				channels[11] =
					((((data_bytes[16] >> 1) | (data_bytes[17] << 7)) as u16) & 0x07FF).into();
				channels[12] =
					((((data_bytes[17] >> 4) | (data_bytes[18] << 4)) as u16) & 0x07FF).into();
				channels[13] =
					((((data_bytes[18] >> 7) | (data_bytes[19] << 1) | (data_bytes[20] << 9)) as u16)
						& 0x07FF)
						.into();
				channels[14] =
					((((data_bytes[20] >> 2) | (data_bytes[21] << 6)) as u16) & 0x07FF).into();
				channels[15] =
					((((data_bytes[21] >> 5) | (data_bytes[22] << 3)) as u16) & 0x07FF).into();

				let flag_byte = self.buffer.pop_front().unwrap_or(0);

				Some(SbusPacket {
					channels,
					digital_channels: [is_flag_set(flag_byte, 0), is_flag_set(flag_byte, 1)],
					frame_lost: is_flag_set(flag_byte, 2),
					failsafe: is_flag_set(flag_byte, 3),
				})
			} else {
				while self.buffer.len() > 0 && *self.buffer.get(0).unwrap() != HEADER {
					self.buffer.pop_front();
				}

				None
			}
		}
	}
}

fn is_flag_set(flag_byte: u8, idx: u8) -> bool {
	flag_byte & 1 << idx == 1
}


#[cfg(test)]
mod tests {
	//TODO
}
