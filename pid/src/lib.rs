#[cfg(test)]
#[macro_use]
extern crate assert_approx_eq;

use std::{fmt::Display, time::Instant};
use dsp::{ScalarAlphaBeta};
use nalgebra::{RealField};

pub struct Pid<N: RealField> {
	k: (N, N, N),
	error_integral: N,
	target: N,
	last_input: Option<(N, Instant)>,
	last_output: Option<N>,
	limits: Option<(N, N)>,
	d_term_filter: ScalarAlphaBeta<N>,
}

impl<N: RealField + From<f64> + Display> Pid<N> {
	pub fn new(k: (N, N, N), target: N, limits: Option<(N, N)>) -> Self {
		Self {
			k,
			error_integral: N::zero(),
			target,
			last_input: None,
			last_output: None,
			limits,
			d_term_filter: ScalarAlphaBeta::new(N::from(0.008), N::from(0.0005)),
		}
	}

	pub fn setpoint(&self) -> N {
		self.target
	}

	pub fn set_setpoint(&mut self, target: N) {
		if target != self.target {
			self.target = target;
			self.reset();
		}
	}

	pub fn reset(&mut self) {
		self.error_integral = N::zero();
		self.last_output = None;
		self.last_input = None;
	}

	fn estimate_with_new_input(&mut self, input: N, input_read_instant: Instant) -> N {
		let error = self.target - input;

		let p: N = self.k.0 * error;

		let (i, d) = {
			if let Some((last_input, last_instant)) = self.last_input {
				let dt: N = (input_read_instant - last_instant).as_secs_f64().into();

				self.error_integral = self.error_integral + self.k.1 * dt * error;

				let filtered_d_term = self
					.d_term_filter
					.update(-self.k.2 * (last_input - input), dt);

				(self.error_integral, filtered_d_term / dt) // Note: d(err)/dt = - d(input)/dt
			} else {
				(N::zero(), N::zero())
			}
		};

		let output = {
			let output = p + i + d;
			if let Some((a, b)) = self.limits {
				output.max(a).min(b)
			} else {
				output
			}
		};

		self.last_output = Some(output);
		self.last_input = Some((input, input_read_instant));

		output
	}

	/// Estimate the best output value to obtain targeted input.
	/// This function can be called on redundant data without crashing. It only computes output
	/// values if the time delta between the latest calculated value's timestamp is strictly
	/// greater than zero.
	pub fn estimate(&mut self, input: N, input_read_instant: Instant) -> N {
		if let Some((_, last_instant)) = self.last_input {
			if last_instant >= input_read_instant {
				return self.last_output.unwrap();
			}
		}

		self.estimate_with_new_input(input, input_read_instant)
	}
}

#[cfg(test)]
mod tests {
	use crate::Pid;
	use std::time::{Duration, Instant};

	const D_100_MS: Duration = Duration::from_millis(100);
	const D_1000_MS: Duration = Duration::from_millis(1000);

	#[test]
	fn target_test() {
		let pid = Pid::<f32>::new((0., 0., 0.), 2.5443, None);
		assert_approx_eq!(pid.target, 2.5443);
	}

	#[test]
	fn no_k_test() {
		let target = 3.5f32;
		let initial_instant = Instant::now();
		let mut pid = Pid::<f32>::new((0., 0., 0.), target, None);

		assert_approx_eq!(pid.estimate(0.0, initial_instant), 0.);
		assert_approx_eq!(pid.estimate(4.3, initial_instant), 0.);
		assert_approx_eq!(pid.estimate(2.3, initial_instant + D_100_MS), 0.);
		assert_approx_eq!(pid.estimate(-2.0, initial_instant + D_1000_MS), 0.);
	}

	#[test]
	fn kp_test() {
		let target = 12.4205;
		let initial_instant = Instant::now();
		let mut pid = Pid::<f32>::new((3.4, 0., 0.), target, None);
		assert_approx_eq!(pid.estimate(4., initial_instant), (target - 4.) * 3.4);
		assert_approx_eq!(
            pid.estimate(-2., initial_instant + D_100_MS),
            (target - (-2.)) * 3.4
        );
	}

	#[test]
	fn ki_test() {
		let target = -5.48694;
		let initial_instant = Instant::now();

		let mut pid = Pid::<f32>::new((0., 0.8, 0.), target, None);
		assert_approx_eq!(pid.estimate(4., initial_instant), 0.);
		let error_integral = (target - 2.) * 0.1 * 0.8;
		assert_approx_eq!(pid.estimate(2., initial_instant + D_100_MS), error_integral);
		assert_approx_eq!(
            pid.estimate(-7., initial_instant + D_1000_MS),
            error_integral + (target - (-7.)) * 0.9 * 0.8
        );
	}

	#[test]
	fn kd_test() {
		let target = 453.246;
		let initial_instant = Instant::now();

		let d = 12.34;
		let mut pid = Pid::<f32>::new((0., 0., d), target, None);
		assert_approx_eq!(pid.estimate(423., initial_instant), 0.);
		assert_approx_eq!(
            pid.estimate(421., initial_instant + D_100_MS),
            -d * (421. - 423.) / 0.1
        );
		assert_approx_eq!(
            pid.estimate(432., initial_instant + D_1000_MS),
            -d * (432. - 421.) / 0.9
        );
	}

	#[test]
	fn pid_test() {
		let target = -0.246;
		let initial_instant = Instant::now();

		let p = 0.964;
		let i = 0.543;
		let d = 0.34;
		let mut pid = Pid::<f32>::new((p, i, d), target, None);

		assert_approx_eq!(pid.estimate(0.0432, initial_instant), p * (target - 0.0432));
		assert_approx_eq!(
            pid.estimate(-0.143, initial_instant + D_100_MS),
            p * (target - (-0.143)) + i * (target - (-0.143)) * 0.1 + d * -(-0.143 - 0.0432) / 0.1
        );

		assert_approx_eq!(
            pid.estimate(-0.248, initial_instant + D_1000_MS),
            p * (target - (-0.248))
                + i * (target - (-0.143)) * 0.1
                + i * (target - (-0.248)) * 0.9
                + d * -(-0.248 - -0.143) / 0.9
        );
	}

	#[test]
	fn limits_test() {
		let target = -0.246;
		let initial_instant = Instant::now();

		let p = 0.964;
		let i = 0.543;
		let d = 0.34;
		let mut pid = Pid::<f32>::new((p, i, d), target, Some((0.04, 0.5)));

		assert_approx_eq!(pid.estimate(0.0432, initial_instant), 0.04);
		assert_approx_eq!(pid.estimate(-0.143, initial_instant + D_100_MS), 0.5);
		assert_approx_eq!(pid.estimate(-0.248, initial_instant + D_1000_MS), 0.04);
	}
}

