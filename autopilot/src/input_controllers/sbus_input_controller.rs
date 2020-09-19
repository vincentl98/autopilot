use crate::input::{InputController, Input, RcChannels};
use crossbeam_channel::Sender;
use std::time::{Duration};
use serialport::{SerialPort, SerialPortSettings, DataBits, FlowControl, Parity, StopBits};


/// Note: the serial input should be inverted with an adequate circuit.
pub struct SbusInputController {
	serial_port: Box<dyn SerialPort>,
}

impl SbusInputController {
	pub fn new() -> anyhow::Result<Self> {
		const SETTINGS: SerialPortSettings = SerialPortSettings {
			baud_rate: 100_000,
			data_bits: DataBits::Eight,
			flow_control: FlowControl::None,
			parity: Parity::Even,
			stop_bits: StopBits::Two,
			timeout: Duration::from_millis(14),
		};

		const SERIAL_PORT: &'static str = "/dev/serial0";
		Ok(SbusInputController {
			serial_port: serialport::open_with_settings(SERIAL_PORT, &SETTINGS)
				.map_err(|e| error!("Failed to open serial port {}: {}", &SERIAL_PORT, e))
				.unwrap(),
		})
	}
}

impl InputController for SbusInputController {
	fn read_input(&mut self, input_sender: Sender<Input>) -> ! {
		let mut buffer = [0u8; sbus::BUFFER_SIZE];

		loop {
			if self.serial_port
				.read_exact(&mut buffer)
				.map_err(|e| error!("Failed to read from serial port: {}", &e))
				.is_ok() {

				if let Some(packet) = sbus::try_parse(&buffer) {
					input_sender
						.send(Input::RcChannels(RcChannels {
							channels: packet.normalized_channels().clone(),
							failsafe: packet.failsafe,
						}))
						.unwrap()
				}
			} else {
				warn!("No bytes read from serial");
			}
		}
	}
}