#![feature(array_map)]

// An SBUS packet is sent every 14 ms. Each packet is made of 25 * 8 = 200 bits. With the SBUS
// baud rate of 100 000 bps, each packet takes 200 / 100 000 = 2 ms to be received. That is a total
// period of 14 + 2 = 16 ms. This can be down-sampled to about 1/2 by setting buffer size to twice
// the packet size. With this buffer size, it is guaranteed to always have one full packet in the
// buffer, while it is also possible to have two.

use std::{
    error::Error,
    fmt::{Display, Formatter},
};

pub const BUFFER_SIZE: usize = 2 * PACKET_SIZE;
pub const PACKET_SIZE: usize = 25;
pub const CHANNELS: usize = 16;

#[derive(Debug, Clone)]
pub struct RawPacket {
    pub channels: [u16; CHANNELS],
    pub digital_channels: [bool; 2],
    pub failsafe: bool,
    pub frame_lost: bool,
}

#[derive(Debug, Clone)]
pub struct Packet {
    pub channels: [f32; CHANNELS],
    pub digital_channels: [bool; 2],
    pub failsafe: bool,
    pub frame_lost: bool,
}

impl Into<Packet> for RawPacket {
    fn into(self) -> Packet {
        Packet {
            channels: self.channels.map(Self::normalized_channel),
            digital_channels: self.digital_channels,
            failsafe: self.failsafe,
            frame_lost: self.frame_lost,
        }
    }
}

impl RawPacket {
    fn normalized_channel(channel: u16) -> f32 {
        // There is no documented max/min values as it is a proprietary protocol. However, these
        // values seem to be the most extreme possible. Theoretical range is 0-2047 as each channel
        // is encoded as 11 bits, but most transmitters won't go below/above the constants below.

        const MIN: u16 = 172;
        const MAX: u16 = 1811;
        const INV_RANGE: f32 = 1. / (MAX - MIN) as f32;

        -1.0 + (channel.min(MAX).max(MIN) - MIN) as f32 * 2.0 * INV_RANGE
    }
}

#[derive(Debug)]
pub struct SbusError {
    error_type: SbusErrorType,
}

#[derive(Debug)]
pub enum SbusErrorType {
    HeaderNotFound,
    InvalidPacker,
}

impl SbusError {
    pub fn new(error_type: SbusErrorType) -> Self {
        Self { error_type }
    }
}

impl Error for SbusError {}

impl Display for SbusError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.write_str(format!("{:?}", self.error_type).as_str())
    }
}

pub fn try_parse(buffer: &[u8; BUFFER_SIZE]) -> Result<RawPacket, SbusError> {
    // The implementation is made as follows:
    // - Given a non-circular buffer, search for the first footer byte beginning by the end of the
    // buffer
    // - Once found, check some parameters: enough size behind the footer to hold a packet, the flag
    // mask is here, and so is the header. Else, it is highly unlikely that another packet is
    // present (only the case when having two packets in the same buffer), so we'll just abort.
    // - Then assume the packet is correct and parse it
    // There is not checksum implemented in this protocol. The serial protocol implements it though,
    // through the parity bit. But on fully random data, the parity bit has 1/2 odd to be correct.

    const FOOTER: u8 = 0b00000000;

    if let Some(footer_rev) = buffer
        .iter()
        .rev() // This iterator could be restrained to only the last half of the buffer
        .position(|&item| item == FOOTER)
    {
        let footer = BUFFER_SIZE - footer_rev;

        const FLAG_MASK: u8 = 0b11110000;
        const HEADER: u8 = 0x0F;
        if footer >= PACKET_SIZE
            && buffer[footer - 1] & FLAG_MASK == 0
            && buffer[footer - PACKET_SIZE] == HEADER
        {
            let header = footer - PACKET_SIZE;
            let mut data_bytes = [0u16; PACKET_SIZE - 2];

            for j in 0..PACKET_SIZE - 2 {
                data_bytes[j] = buffer[header + j] as u16;
            }

            let mut channels = [0u16; 16];

            const BIT_MASK: u16 = 0x07FF;

            channels[0] = (data_bytes[1]) | (data_bytes[2] << 8) & BIT_MASK;
            channels[1] = (data_bytes[2] >> 3) | (data_bytes[3] << 5) & BIT_MASK;
            channels[2] =
                (data_bytes[3] >> 6) | (data_bytes[4] << 2) | (data_bytes[5] << 10) & BIT_MASK;
            channels[3] = (data_bytes[5] >> 1) | (data_bytes[6] << 7) & BIT_MASK;
            channels[4] = (data_bytes[6] >> 4) | (data_bytes[7] << 4) & BIT_MASK;
            channels[5] =
                (data_bytes[7] >> 7) | (data_bytes[8] << 1) | (data_bytes[9] << 9) & BIT_MASK;
            channels[6] = (data_bytes[9] >> 2) | (data_bytes[10] << 6) & BIT_MASK;
            channels[7] = (data_bytes[10] >> 5) | (data_bytes[11] << 3) & BIT_MASK;
            channels[8] = (data_bytes[12]) | (data_bytes[13] << 8) & BIT_MASK;
            channels[9] = (data_bytes[13] >> 3) | (data_bytes[14] << 5) & BIT_MASK;
            channels[10] =
                (data_bytes[14] >> 6) | (data_bytes[15] << 2) | (data_bytes[16] << 10) & BIT_MASK;
            channels[11] = (data_bytes[16] >> 1) | (data_bytes[17] << 7) & BIT_MASK;
            channels[12] = (data_bytes[17] >> 4) | (data_bytes[18] << 4) & BIT_MASK;
            channels[13] =
                (data_bytes[18] >> 7) | (data_bytes[19] << 1) | (data_bytes[20] << 9) & BIT_MASK;
            channels[14] = (data_bytes[20] >> 2) | (data_bytes[21] << 6) & BIT_MASK;
            channels[15] = (data_bytes[21] >> 5) | (data_bytes[22] << 3) & BIT_MASK;

            let flag_byte = buffer[23];

            Ok(RawPacket {
                channels,
                digital_channels: [is_flag_set(flag_byte, 0), is_flag_set(flag_byte, 1)],
                frame_lost: is_flag_set(flag_byte, 2),
                failsafe: is_flag_set(flag_byte, 3),
            })
        } else {
            Err(SbusError::new(SbusErrorType::InvalidPacker))
        }
    } else {
        Err(SbusError::new(SbusErrorType::HeaderNotFound))
    }
}

/// TODO: use `bitflags`
#[inline(always)]
fn is_flag_set(flag_byte: u8, idx: u8) -> bool {
    flag_byte & 1 << idx == 1
}

#[cfg(test)]
mod tests {
    //TODO
}
