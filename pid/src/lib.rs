#[cfg(test)]
#[macro_use]
extern crate assert_approx_eq;

use num_traits::float::FloatCore;
use std::{fmt::Display, time::Instant};

pub struct Pid<F: FloatCore> {
    k: (F, F, F),
    error_integral: F,
    target: F,
    last_input: Option<(F, Instant)>,
    last_output: Option<F>,
    /// Error returned should be between -1 and 1 inclusive.
    error: fn(F, F) -> F,
    limits: Option<(F, F)>,
}

impl<F: FloatCore + From<f32> + Display> Pid<F> {
    const DEFAULT_ERROR: fn(F, F) -> F = |input, target| target - input;

    pub fn with_custom_error(
        k: (F, F, F),
        target: F,
        limits: Option<(F, F)>,
        error: fn(F, F) -> F,
    ) -> Self {
        Self {
            k,
            error_integral: F::zero(),
            target,
            last_input: None,
            last_output: None,
            error,
            limits,
        }
    }

    pub fn new(k: (F, F, F), target: F, limits: Option<(F, F)>) -> Self {
        Self {
            k,
            error_integral: F::zero(),
            target,
            last_input: None,
            last_output: None,
            error: Self::DEFAULT_ERROR,
            limits,
        }
    }

    pub fn target(&self) -> F {
        self.target
    }

    pub fn set_target(&mut self, target: F) {
        self.target = target;
        self.reset();
    }

    fn reset(&mut self) {
        self.error_integral = F::zero();
        self.last_output = None;
        self.last_input = None;
    }

    /// It is guaranteed to have a strictly positive time delta
    fn estimate_with_new_input(&mut self, input: F, input_read_instant: Instant) -> F {
        let error = (self.error)(input, self.target);

        let p: F = self.k.0 * error;

        let (i, d) = {
            if let Some((last_input, last_instant)) = self.last_input {
                let dt: F = (input_read_instant - last_instant).as_secs_f32().into();

                assert!(dt != F::zero());
                self.error_integral = self.error_integral + self.k.1 * dt * error;

                // TODO: Maybe use - d(input)/dt rather than d(err)/dt
                (self.error_integral, self.k.2 * (last_input - input) / dt) // Note: d(err)/dt = - d(input)/dt
            } else {
                (F::zero(), F::zero())
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

        // error!("Error is {:.2} (hdg: {:.2}, tgt: {:.2}), P+I+D = {:.2}, P={:.2}, I={:.2}, D={:.2}", error, input, self.target, output, p, i, d);

        self.last_output = Some(output);
        self.last_input = Some((input, input_read_instant));

        output
    }

    /// Estimate the best output value to obtain targeted input.
    /// This function can be called on redundant data without crashing. It only computes output
    /// values if the time delta between the latest calculated value's timestamp is strictly
    /// greater than zero.
    pub fn estimate(&mut self, input: F, input_read_instant: Instant) -> F {
        if let Some((_, last_instant)) = self.last_input {
            if last_instant >= input_read_instant {
                self.last_output.unwrap()
            } else {
                self.estimate_with_new_input(input, input_read_instant)
            }
        } else {
            self.estimate_with_new_input(input, input_read_instant)
        }
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
