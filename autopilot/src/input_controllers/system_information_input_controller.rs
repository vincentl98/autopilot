use crossbeam_channel::Sender;
use systemstat::Platform;
use std::thread;
use std::time::Duration;
use crate::input::{InputController, Input};


#[derive(Debug, Copy, Clone)]
pub struct SystemInformation {
	pub cpu_temperature: f32,
	pub cpu_load_one_minute: f32,
	pub available_memory_mib: u64,
	pub uptime: Duration,
}

impl ToString for SystemInformation {
	fn to_string(&self) -> String {
		format!("CPU load (1 min): {:.1}%, \
		Available memory: {:.0} MiB, \
		Uptime: {} s, \
		CPU temperature: {:.1}°C",
				self.cpu_load_one_minute,
				self.available_memory_mib,
				self.uptime.as_secs_f32(),
				self.cpu_temperature)
	}
}

impl SystemInformation {
	pub const MIB: u64 = 1_048_576;
}


pub struct SystemInformationInputController {
	system: systemstat::System,
}

impl SystemInformationInputController {
	pub fn new() -> Self {
		SystemInformationInputController {
			system: systemstat::System::new()
		}
	}
}

impl InputController for SystemInformationInputController {
	fn read_input(&mut self, input_sender: Sender<Input>) -> ! {
		loop {
			let system_information = SystemInformation {
				cpu_temperature: self.system.cpu_temp()
					.map_err(|e| error!("CPU temperature not available: {}", e)).unwrap(),
				cpu_load_one_minute: self.system.load_average()
					.map_err(|e| error!("CPU load average not available: {}", e)).unwrap()
					.one,
				available_memory_mib: self.system.memory()
					.map_err(|e| error!("Available memory not available: {}", e)).unwrap()
					.free.as_u64() / SystemInformation::MIB,
				uptime: self.system.uptime()
					.map_err(|e| error!("Uptime not available: {}", e)).unwrap(),
			};

			input_sender.send(Input::SystemInformation(system_information)).unwrap();

			const LOOP_DELAY: Duration = Duration::from_secs(5);
			thread::sleep(LOOP_DELAY);
		}
	}
}