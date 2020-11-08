use nalgebra::{Vector3, RealField, zero};

pub struct Biquad<N: RealField> {
	a: (N, N, N),
	b: (N, N, N),
	input_state: (Vector3<N>, Vector3<N>),
	output_state: (Vector3<N>, Vector3<N>),
}

pub fn low_pass(f: f64, q: f64, sample_frequency: f64) -> Biquad<f64> {
	let omega = 2.0 * std::f64::consts::PI * f / sample_frequency;

	let omega_s = omega.sin();
	let omega_c = omega.cos();
	let alpha = omega_s / (2.0 * q);

	let b0 = (1.0 - omega_c) * 0.5;
	let b1 = 1.0 - omega_c;
	let b2 = (1.0 - omega_c) * 0.5;

	let a0 = 1.0 + alpha;
	let a1 = -2.0 * omega_c;
	let a2 = 1.0 - alpha;

	Biquad::<f64>::new((a0, a1 / a0, a2 / a0), (b0 / a0, b1 / a0, b2 / a0))
}

impl<N: RealField> Biquad<N> where Self: Send + Sync {
	pub fn new(a: (N, N, N), b: (N, N, N)) -> Self {
		Self {
			a,
			b,
			input_state: (zero(), zero()),
			output_state: (zero(), zero()),
		}
	}

	pub fn update(&mut self, input: Vector3<N>) -> Vector3<N> {
		let y = input.scale(self.b.0)
			+ self.input_state.0.scale(self.b.1)
			+ self.input_state.1.scale(self.b.2)
			- self.output_state.0.scale(self.a.1)
			- self.output_state.1.scale(self.a.2);

		self.input_state.1 = self.input_state.0.clone();
		self.input_state.0 = input;

		self.output_state.1 = self.output_state.0.clone();
		self.output_state.0 = y.clone();

		y
	}
}