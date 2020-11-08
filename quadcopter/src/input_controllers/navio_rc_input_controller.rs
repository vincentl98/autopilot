use autopilot::{InputController, Input};
use std::error::Error;
use systemstat::Duration;
use std::io;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};

pub struct NavioRcInputController {
	channel_files: [File; CHANNELS],
	connected_file: File,
	range: (u16, u16),
	inv_range: f64,
}

const RC_IN_PATH: &'static str = "/sys/kernel/rcio/rcin";
pub const CHANNELS: usize = 16;
// pub const FLYSKY_RANGE: (u16, u16) = (1024, 2003);

impl NavioRcInputController {
	pub fn new(range: (u16, u16)) -> Result<Self, io::Error> {
		let channel_files = [
			File::open(format!("{}/ch0", RC_IN_PATH))?,
			File::open(format!("{}/ch1", RC_IN_PATH))?,
			File::open(format!("{}/ch2", RC_IN_PATH))?,
			File::open(format!("{}/ch3", RC_IN_PATH))?,
			File::open(format!("{}/ch4", RC_IN_PATH))?,
			File::open(format!("{}/ch5", RC_IN_PATH))?,
			File::open(format!("{}/ch6", RC_IN_PATH))?,
			File::open(format!("{}/ch7", RC_IN_PATH))?,
			File::open(format!("{}/ch8", RC_IN_PATH))?,
			File::open(format!("{}/ch9", RC_IN_PATH))?,
			File::open(format!("{}/ch10", RC_IN_PATH))?,
			File::open(format!("{}/ch11", RC_IN_PATH))?,
			File::open(format!("{}/ch12", RC_IN_PATH))?,
			File::open(format!("{}/ch13", RC_IN_PATH))?,
			File::open(format!("{}/ch14", RC_IN_PATH))?,
			File::open(format!("{}/ch15", RC_IN_PATH))?,
		];

		let connected_file = File::open(format!("{}/connected", RC_IN_PATH))?;

		Ok(Self {
			channel_files,
			connected_file,
			range,
			inv_range: 1. / (range.1 - range.0) as f64,
		})
	}

	fn channel_parse(file: &mut File) -> Result<u16, Box<dyn Error>> {
		let mut s = String::with_capacity(10);
		file.read_to_string(&mut s)?;
		file.seek(SeekFrom::Start(0))?;
		Ok(s.trim().parse::<u16>()?)
	}

	fn normalized_channel(&self, channel: u16) -> f64 {
		// There is no documented max/min values as it is a proprietary protocol. However, these
		// values seem to be the most extreme possible. Theoretical range is 0-2047 as each channel
		// is encoded as 11 bits, but most transmitters won't go below/above the constants below.

		// const MIN: u16 = 172;
		// const MAX: u16 = 1811;

		(channel.min(self.range.1).max(self.range.0) - self.range.0) as f64 * self.inv_range
	}

}

impl InputController for NavioRcInputController {
	const DELAY: Option<Duration> = Some(Duration::from_millis(20));

	fn read_input(&mut self) -> Result<Input, Box<dyn Error>> {
		let mut channels = [0.; CHANNELS];

		for i in 0..CHANNELS {
			let raw_channel = Self::channel_parse(&mut self.channel_files[i])?;
			channels[i] = self.normalized_channel(raw_channel);
		}

		let connected = {
			let connected = Self::channel_parse(&mut self.connected_file)?;
			match connected {
				1 => true,
				_ => false,
			}
		};

		info!(target: "navio_rc", "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
			  connected as u8,
			  channels[0], channels[1], channels[2], channels[3],
			  channels[4], channels[5], channels[6], channels[7],
			  channels[8], channels[9], channels[10], channels[11],
			  channels[12], channels[13], channels[14], channels[15]);

		if connected {
			Ok(Input::RcChannels(Some(channels)))
		} else {
			Ok(Input::RcChannels(None))
		}
	}
}