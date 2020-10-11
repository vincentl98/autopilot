use std::{
    error::Error,
    fmt,
    fmt::{Display, Formatter},
    time::Duration,
};
use systemstat::Platform;
use autopilot::Monitor;

#[derive(Debug, Copy, Clone)]
pub struct SystemInformation {
    pub cpu_temperature: f32,
    pub cpu_load_one_minute: f32,
    pub available_memory_mib: u64,
    pub uptime: Duration,
}

impl SystemInformation {
    pub const MIB: u64 = 1_048_576;
}

impl Display for SystemInformation {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        Ok(write!(
            f,
            "CPU load (1 min): {:.1}%, \
		Available memory: {:.0} MiB, \
		Uptime: {} s, \
		CPU temperature: {:.1}°C",
            self.cpu_load_one_minute,
            self.available_memory_mib,
            self.uptime.as_secs_f32(),
            self.cpu_temperature
        )?)
    }
}

pub struct SystemInformationMonitor {
    system: systemstat::System,
}

impl SystemInformationMonitor {
    pub fn new() -> Self {
        SystemInformationMonitor {
            system: systemstat::System::new(),
        }
    }
}

impl Monitor<SystemInformation> for SystemInformationMonitor {
    const DELAY: Option<Duration> = Some(Duration::from_secs(5));

    fn monitor(&mut self) -> Result<SystemInformation, Box<dyn Error>> {
        let system_information = SystemInformation {
            cpu_temperature: self.system.cpu_temp()?,
            cpu_load_one_minute: self.system.load_average()?.one,
            available_memory_mib: self.system.memory()?.free.as_u64() / SystemInformation::MIB,
            uptime: self.system.uptime()?,
        };

        Ok(system_information)
    }
}
