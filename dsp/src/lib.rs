mod alpha_beta_gamma;
pub mod biquad;

pub use alpha_beta_gamma::AlphaBetaGamma;
pub use alpha_beta_gamma::ScalarAlphaBeta;
pub use biquad::Biquad;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
