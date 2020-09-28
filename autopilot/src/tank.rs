use crossbeam_channel::Sender;
use std::time::Instant;

use navigation;
use pid::Pid;

use crate::{
    input::Input,
    traits::{Autopilot, Collector, Dispatcher},
};
use systemstat::Duration;

#[derive(Debug, Clone, Default)]
pub struct TankInputFrame {
    pub orientation: Option<(mint::EulerAngles<f32, ()>, Instant)>,
    pub rc_channels: Option<sbus::Packet>,
    pub soft_armed: Option<bool>,
}

#[derive(Debug)]
pub struct TankOutputFrame {
    pub left_motor: f32,
    pub right_motor: f32,
}

impl TankOutputFrame {
    pub fn normalized(self) -> Self {
        let left = self.left_motor;
        let right = self.right_motor;

        let normalization_factor = 0.5 / (left * left + right * right).sqrt();

        let left_sign = left.signum();
        let right_sign = right.signum();

        let left_abs = left.abs() * 1f32.min(normalization_factor);
        let right_abs = right.abs() * 1f32.min(normalization_factor);

        const THRESHOLD: f32 = 0.; // TMP
        TankOutputFrame {
            left_motor: {
                if left_abs >= THRESHOLD {
                    left_sign * left_abs
                } else {
                    0.
                }
            },
            right_motor: {
                if right_abs >= THRESHOLD {
                    right_sign * right_abs
                } else {
                    0.
                }
            },
        }
    }
}

impl Default for TankOutputFrame {
    fn default() -> Self {
        Self {
            left_motor: 0f32,
            right_motor: 0f32,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    ArmedControlled,
    ArmedAutonomous,
    Disarmed,
    Failsafe,
}

pub struct TankAutopilot {
    heading_pid: Pid<f32>,
    last_mode: Option<Mode>,
}

impl TankAutopilot {
    pub fn new(pid: (f32, f32, f32)) -> Self {
        Self {
            heading_pid: Pid::<f32>::with_custom_error(pid, 0.0, Some((-1.0, 1.0)), angle_error_fn),
            last_mode: None,
        }
    }
}

impl Autopilot<TankInputFrame, TankOutputFrame> for TankAutopilot {
    const MAX_CONTROL_LOOP_PERIOD: Duration = Duration::from_millis(50);

    fn output_frame(&mut self, input_frame: TankInputFrame) -> TankOutputFrame {
        let mode = {
            // There are three levels of arming: software (this program is running), rc controlled
            // and autonomous
            const TRIGGER_A: usize = 4;
            const TRIGGER_B: usize = 5;

            match (&input_frame.soft_armed, &input_frame.rc_channels) {
                (Some(soft_armed), Some(rc_channels)) => {
                    if rc_channels.failsafe {
                        Mode::Failsafe
                    } else {
                        if *soft_armed && rc_channels.channels[TRIGGER_A] > 0. {
                            if rc_channels.channels[TRIGGER_B] > 0. {
                                Mode::ArmedAutonomous
                            } else {
                                Mode::ArmedControlled
                            }
                        } else {
                            Mode::Disarmed
                        }
                    }
                }
                _ => Mode::Disarmed,
            }
        };

        info!("Mode: {:?}", &mode);

        let result = match mode {
            Mode::ArmedAutonomous => {
                let rc_channels = input_frame.rc_channels.expect("Empty RC channels");

                let power = rc_channels.channels[1];

                match input_frame.orientation {
                    Some((orientation, instant)) => {
                        let heading = orientation.a;

                        let target = (rc_channels.channels[8] + 1.) * 180.;

                        if let Some(last_mode) = self.last_mode {
                            if mode != last_mode {
                                self.heading_pid.set_target(target);
                            }
                        }

                        let estimated = self.heading_pid.estimate(heading, instant);

                        TankOutputFrame {
                            left_motor: power + estimated,
                            right_motor: power - estimated,
                        }
                        .normalized()
                    }
                    None => TankOutputFrame::default(),
                }
            }
            Mode::ArmedControlled => {
                let rc_channels = input_frame.rc_channels.expect("Empty RC channels");

                info!("RC channels: {:?}", rc_channels);

                let direction = rc_channels.channels[0];
                let power = rc_channels.channels[1];

                TankOutputFrame {
                    left_motor: power + direction,
                    right_motor: power - direction,
                }
                .normalized()
            }
            Mode::Disarmed => TankOutputFrame::default(),
            Mode::Failsafe => TankOutputFrame::default(),
        };

        self.last_mode = Some(mode);
        result
    }
}

fn angle_error_fn(input: f32, target: f32) -> f32 {
    let directed_angle = navigation::angle_difference_deg(input as i32, target as i32);
    directed_angle as f32 / 180.
}

pub struct TankDispatcher {
    pub motors_sender: Sender<(f64, f64)>,
}

impl Dispatcher<TankOutputFrame> for TankDispatcher {
    fn dispatch(&self, output_frame: TankOutputFrame) {
        let motors = (
            output_frame.left_motor as f64,
            output_frame.right_motor as f64,
        );

        self.motors_sender.send(motors).unwrap()
    }
}

pub struct TankCollector {
    input_frame: TankInputFrame,
}

impl TankCollector {
    pub fn new() -> Self {
        Self {
            input_frame: TankInputFrame::default(),
        }
    }
}

impl Collector<TankInputFrame> for TankCollector {
    fn collect(&mut self, input: Input) -> TankInputFrame {
        match input {
            Input::RcChannels(rc_channels) => self.input_frame.rc_channels = Some(rc_channels),
            Input::SoftArmed(armed) => self.input_frame.soft_armed = Some(armed),
            Input::OrientationEuler(orientation) => {
                self.input_frame.orientation = Some(orientation)
            }
            _ => warn!("Unhandled input: {:?}", input),
        }
        self.input_frame.clone()
    }
}
