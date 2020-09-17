use std::time::Duration;

pub struct ServoSpecifications {
	pub min_pulse_width: Duration,
	pub max_pulse_width: Duration,
}

pub const SG90: ServoSpecifications = ServoSpecifications {
	min_pulse_width: Duration::from_micros(700),
	max_pulse_width: Duration::from_micros(2800),
};

pub const CS929MG: ServoSpecifications = ServoSpecifications {
	min_pulse_width: Duration::from_micros(1950),
	max_pulse_width: Duration::from_micros(650),
};
