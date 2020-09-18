use crate::input::{InputController, Input, RcChannels};
use crossbeam_channel::Sender;
use std::time::{Duration};
use serialport::{SerialPort, SerialPortSettings, DataBits, FlowControl, Parity, StopBits, ClearBuffer};
use sbus::SbusPacketParser;
use std::thread;


/// Note: the serial input should be inverted with an adequate circuit.
pub struct SbusInputController {
	serial_port: Box<dyn SerialPort>,
	packet_parser: SbusPacketParser,
}

impl SbusInputController {
	pub fn new() -> anyhow::Result<Self> {
		const SETTINGS: SerialPortSettings = SerialPortSettings {
			baud_rate: 100_000,
			data_bits: DataBits::Eight,
			flow_control: FlowControl::None,
			parity: Parity::Even,
			stop_bits: StopBits::Two,
			timeout: Duration::from_secs(5),
		};

		const SERIAL_PORT: &'static str = "/dev/serial0";
		Ok(SbusInputController {
			serial_port: serialport::open_with_settings(SERIAL_PORT, &SETTINGS)
				.map_err(|e| error!("Failed to open serial port {}: {}", &SERIAL_PORT, e))
				.unwrap(),
			packet_parser: SbusPacketParser::new(),
		})
	}
}

impl InputController for SbusInputController {
	fn read_input(&mut self, input_sender: Sender<Input>) -> ! {
		const SBUS_PACKET_SIZE: usize = 25;

		let mut buffer = [0u8; SBUS_PACKET_SIZE];

		loop {
			let read_bytes = self.serial_port
				.read(&mut buffer)
				.map_err(|e| error!("Failed to read from serial port: {}", &e))
				.unwrap_or(0);

			if read_bytes > 0 {
				self.packet_parser.push_bytes(&buffer[..read_bytes]);

				if let Some(packet) = self.packet_parser.try_parse() {
					input_sender
						.send(Input::RcChannels(RcChannels {
							channels: packet.normalized_channels().clone(),
							failsafe: false,
						}))
						.unwrap()
				}
			} else {
				warn!("No bytes read from serial");
			}

			// thread::sleep(Duration::from_millis(50));
			self.serial_port
				.clear(ClearBuffer::Input)
				.map_err(|e| error!("Failed to clear serial buffer: {}", e))
				.unwrap_or(());

			const LOOP_DELAY: Duration = Duration::from_millis(10);
			thread::sleep(LOOP_DELAY);
		}
	}
}