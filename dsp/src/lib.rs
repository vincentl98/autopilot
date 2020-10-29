use nalgebra::{Vector3, RealField, zero};

pub struct AlphaBetaFilter<N: RealField> {
    alpha: N,
    beta: N,
    state: (Vector3<N>, Vector3<N>),
}

impl<N: RealField> AlphaBetaFilter<N> {
    pub fn new(alpha: N, beta: N) -> Self {
        Self {
            alpha,
            beta,
            state: (zero(), zero())
        }
    }

    pub fn update(&mut self, value: Vector3<N>, dt: N) -> Vector3<N> {
        let (prev_estimated_value, prev_estimated_derivative) = &self.state;
        let estimated_value = prev_estimated_value + prev_estimated_derivative.scale(dt);
        let error = value - estimated_value;

        let estimated_value = estimated_value + error.scale(self.alpha);
        let estimated_derivative = prev_estimated_derivative + error.scale(self.beta / dt);

        self.state = (estimated_value, estimated_derivative);

        estimated_value
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
