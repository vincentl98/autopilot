use chrono::{Datelike, Timelike};
use crossbeam_channel::{unbounded, Receiver, Sender};
use log::{Level, LevelFilter, Log, Metadata, Record};
use std::{
	collections::VecDeque,
	fs::{File, OpenOptions},
	io::Write,
	thread,
	thread::JoinHandle,
	time::{Instant, Duration},
};
use std::error::Error;

lazy_static! {
    static ref BLACK_BOX_CHANNEL: (Sender<BlackBoxInput>, Receiver<BlackBoxInput>) = unbounded::<BlackBoxInput>();
    static ref BLACK_BOX_LOGGER: BlackBoxLogger = BlackBoxLogger {
        start_instant: Instant::now()
    };
}

enum BlackBoxInput {
	Message(String),
	Flush,
}

/// Thread that receives log messages and save them to file asynchronously
pub struct BlackBox {
	file: File,
	buffer: VecDeque<String>,
	last_flush_instant: Instant,
}

impl BlackBox {
	pub fn new() -> Self {
		let log_file_name = {
			let now = chrono::offset::Local::now();

			format!(
				"autopilot_{}-{}-{}_{}-{}-{}.log",
				now.time().hour(),
				now.time().minute(),
				now.time().second(),
				now.date().day(),
				now.date().month(),
				now.date().year()
			)
		};

		BlackBox {
			buffer: VecDeque::<String>::new(),
			file: OpenOptions::new()
				.write(true)
				.create(true)
				.truncate(true)
				.open(log_file_name)
				.unwrap(),
			last_flush_instant: Instant::now(),
		}
	}

	fn try_flush(&mut self) {
		match self.flush() {
			Ok(()) => self.last_flush_instant = Instant::now(),
			Err(e) => {
				self.buffer
					.push_back(format!("Failed to flush black box: {}", e));
			}
		}
	}

	fn flush(&mut self) -> Result<(), Box<dyn Error>> {
		while let Some(message) = self.buffer.pop_front() {
			writeln!(self.file, "{}", message)?;
		}
		Ok(())
	}

	fn receive_loop(&mut self) {
		const RECEIVE_TIMEOUT: Duration = Duration::from_millis(500);

		while let Ok(message) = BLACK_BOX_CHANNEL.1.recv_timeout(RECEIVE_TIMEOUT) {
			match message {
				BlackBoxInput::Message(message) => {
					println!("{}", &message);
					self.buffer.push_back(message)
				}
				BlackBoxInput::Flush => self.try_flush(),
			}

			const MAX_BUFFER_LEN: usize = 64;
			if self.buffer.len() > MAX_BUFFER_LEN {
				self.try_flush();
			}
		}

		if self.buffer.len() > 0 {
			self.try_flush();
		}
	}

	pub fn spawn(mut self, level_filter: LevelFilter) -> JoinHandle<()> {
		log::set_logger(&*BLACK_BOX_LOGGER)
			.map(|()| log::set_max_level(level_filter))
			.unwrap();

		thread::spawn(move || loop { self.receive_loop() })
	}
}

struct BlackBoxLogger {
	start_instant: Instant,
}

/// Log implementation that transmit log messages through channel
impl Log for BlackBoxLogger {
	fn enabled(&self, _: &Metadata) -> bool {
		true
	}

	fn log(&self, record: &Record) {
		if self.enabled(record.metadata()) {
			let formatted_message = {
				if record.metadata().level() == Level::Error {
					format!(
						"[{:.3}][{}] Error: {} ({:?}:{:?})",
						(Instant::now() - self.start_instant).as_secs_f32(),
						record.target(),
						record.args(),
						record.file_static().unwrap_or("unknown"),
						record.line().unwrap_or(0)
					)
				} else {
					format!("[{:.3}][{}] {}",
							(Instant::now() - self.start_instant).as_secs_f32(),
							record.target(),
							record.args())
				}
			};

			BLACK_BOX_CHANNEL.0.send(BlackBoxInput::Message(formatted_message)).unwrap();
		}
	}

	fn flush(&self) {
		BLACK_BOX_CHANNEL.0.send(BlackBoxInput::Flush).unwrap();
	}
}
