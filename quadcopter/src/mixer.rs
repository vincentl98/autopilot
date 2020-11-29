use crate::roll_pitch_yaw::RollPitchYaw;

pub struct Mixer<N> {
	pub min_output: N,
}

impl Mixer<f64> {
	pub fn mix(&self, pid_outputs: RollPitchYaw<f64>, throttle: f64) -> [f64; 4] {
		debug!(target: "mixer_input", "{} {} {} {}",
			   pid_outputs.roll,
			   pid_outputs.pitch,
			   pid_outputs.yaw,
			   throttle);

		// PID outputs are in (-1., +1.), therefore mixed outputs are in (-3., +3.)
		let outputs = [
			pid_outputs.pitch - pid_outputs.roll + pid_outputs.yaw,
			-pid_outputs.pitch - pid_outputs.roll - pid_outputs.yaw,
			-pid_outputs.pitch + pid_outputs.roll + pid_outputs.yaw,
			pid_outputs.pitch + pid_outputs.roll - pid_outputs.yaw
		];

		// Mixed outputs should be normalized before applying throttle, because stabilization is a
		// priority on throttle control
		const ONE_SIXTH: f64 = 1. / 6.;
		let outputs = outputs.map(|x| (x + 3.) * ONE_SIXTH); // (0.0, +1.0)

		// The remaining power is left for throttle control, to a maximum of 1. - max(outputs)
		let max_output = outputs[0]
			.max(outputs[1])
			.max(outputs[2])
			.max(outputs[3]);

		let max_throttle = 1. - max_output;
		let throttle = throttle.min(max_throttle);

		// After throttle is added, the minimum output value is ensured
		let outputs = outputs.map(|x| (x + throttle)
			.max(self.min_output)
			.min(1.));

		debug!(target: "mixer_output", "{} {} {} {}",
			   outputs[0],
			   outputs[1],
			   outputs[2],
			   outputs[3]);

		outputs
	}
}
