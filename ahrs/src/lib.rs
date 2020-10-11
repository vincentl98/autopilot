mod madgwick_ahrs;

pub use madgwick_ahrs::MadgwickAhrs;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
