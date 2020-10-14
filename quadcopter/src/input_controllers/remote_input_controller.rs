use autopilot::{InputController, Input, RemoteCommand};
use std::error::Error;
use systemstat::Duration;
use std::io;
use std::net::{TcpListener, TcpStream};
use serde_json;
use std::io::Read;
use serde::Deserialize;

pub struct RemoteInputController {
	tcp_listener: TcpListener,
	tcp_stream: Option<TcpStream>,
}

impl RemoteInputController {
	pub fn new(port: u16) -> Result<Self, io::Error> {
		let listener = TcpListener::bind(format!("localhost:{}", port))?;

	}
}


impl InputController for RemoteInputController {
	const DELAY: Option<Duration> = None;

	fn read_input(&mut self) -> Result<Input, Box<dyn Error>> {
		println!("loop");

		while self.tcp_stream.is_none() {
			if let Ok((tcp_stream, _)) = self.tcp_listener.accept() {
				self.tcp_stream = Some(tcp_stream);
			} else {
				error!("Failed to accept TCP connection");
			}
		}

		let mut de = serde_json::Deserializer::from_reader(self.tcp_stream.unwrap());
		let x = RemoteCommand::deserialize()
		
		for message in de.into_iter() {

		}
		serde_json::from_reader()
		Ok()
	}
}