use std::time::{Duration, Instant};

use bno055::{Bno055, CalibrationProfile, OperationMode};
use rppal::{hal::Delay, i2c::I2c};

use crate::{input::Input, traits::InputController};
use std::error::Error;

pub struct Bno055InputController {
    bno055: Bno055<I2c>,
}

impl Bno055InputController {
    pub fn new(i2c: I2c) -> anyhow::Result<Self> {
        Ok(Self {
            bno055: Bno055::<I2c>::new(i2c),
        })
    }

    pub fn init(mut self, calibration_profile: Option<CalibrationProfile>) -> anyhow::Result<Self> {
        self.bno055
            .init(OperationMode::NDOF, calibration_profile, &mut Delay {})
            .map_err(|e| anyhow!("{:?}", e))?;

        Ok(self)
    }
}

impl InputController for Bno055InputController {
    // Data output rate is 100 Hz ie every 10 ms, cf datasheet
    const DELAY: Option<Duration> = Some(Duration::from_millis(10));

    fn read_input(&mut self) -> Result<Input, Box<dyn Error>> {
        let euler_angles = self
            .bno055
            .euler_angles()
            .map_err(|e| anyhow!("bno055: {:?}", e))
            .map(|angles| Input::OrientationEuler((angles, Instant::now())))?;

        Ok(euler_angles)
    }
}
