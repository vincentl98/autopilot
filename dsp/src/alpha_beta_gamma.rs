use nalgebra::{Vector3, RealField, zero};

pub struct AlphaBetaGamma<N: RealField> {
	alpha: N,
	beta: N,
	gamma: N,
	state: (Vector3<N>, Vector3<N>, Vector3<N>),
}

impl AlphaBetaGamma<f64> where Self: Send + Sync {
	pub fn new(alpha: f64, beta: f64, gamma: f64) -> Self {
		AlphaBetaGamma::<f64> {
			alpha,
			beta,
			gamma,
			state: (zero(), zero(), zero()),
		}
	}

	pub fn update(&mut self, value: Vector3<f64>, dt: f64) -> Vector3<f64> {
		let (prev_estimated_value,
			prev_estimated_derivative,
			prev_estimated_acceleration) = &self.state;

		let estimated_value = prev_estimated_value
			+ prev_estimated_derivative.scale(dt)
			+ prev_estimated_acceleration.scale(dt * dt);

		let error = value - estimated_value;

		let estimated_value = estimated_value + error.scale(self.alpha);
		let estimated_derivative = prev_estimated_derivative + error.scale(self.beta / dt);
		let estimated_acceleration = prev_estimated_acceleration + error.scale(2.0 * self.gamma / (dt * dt));

		self.state = (estimated_value, estimated_derivative, estimated_acceleration);

		estimated_value
	}
}


pub struct ScalarAlphaBeta<N: RealField> {
	alpha: N,
	beta: N,
	state: (N, N),
}

impl<N: RealField> ScalarAlphaBeta<N> {
	pub fn new(alpha: N, beta: N) -> Self {
		Self {
			alpha, beta,
			state: (zero(), zero())
		}
	}


	pub fn update(&mut self, value: N, dt: N) -> N {
		let (prev_estimated_value,
			prev_estimated_derivative) = self.state;

		let estimated_value = prev_estimated_value
			+ prev_estimated_derivative.scale(dt);

		let error = value - estimated_value;

		let estimated_value = estimated_value + error.scale(self.alpha);
		let estimated_derivative = prev_estimated_derivative + error.scale(self.beta / dt);

		self.state = (estimated_value, estimated_derivative);

		estimated_value
	}
}