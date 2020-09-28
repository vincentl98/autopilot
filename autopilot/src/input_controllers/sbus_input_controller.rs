use crate::{input::Input, traits::InputController};
use sbus::BUFFER_SIZE;
use serialport::{DataBits, FlowControl, Parity, SerialPort, SerialPortSettings, StopBits};
use std::{error::Error, time::Duration};

/// Note: the serial input should be inverted with an adequate circuit.
pub struct SbusInputController {
    serial_port: Box<dyn SerialPort>,
    buffer: [u8; BUFFER_SIZE],
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
                .map_err(|e| anyhow!("Failed to open serial port {}: {:?}", &SERIAL_PORT, e))
                .unwrap(),
            buffer: [0u8; BUFFER_SIZE],
        })
    }
}

impl InputController for SbusInputController {
    // As reading serial port will put the thread in sleep, waiting is not needed
    const DELAY: Option<Duration> = None;

    fn read_input(&mut self) -> Result<Input, Box<dyn Error>> {
        self.serial_port.read_exact(&mut self.buffer)?;

        Ok(Input::RcChannels(sbus::try_parse(&self.buffer)?.into()))
    }
}
