use crossbeam_channel::Sender;
use num_traits::Zero;
use std::time::Instant;

use crate::autopilot::Autopilot;
use crate::collector::{Collector, InputFrame};
use crate::dispatcher::{Dispatcher, OutputFrame};
use crate::input::Input;
use pid::Pid;

type F = f32;

#[derive(Debug)]
pub struct TankOutputFrame {
    pub left_motor: F,
    pub right_motor: F,
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
            left_motor: F::zero(),
            right_motor: F::zero(),
        }
    }
}

impl OutputFrame for TankOutputFrame {}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    ArmedControlled,
    ArmedAutonomous,
    Disarmed,
    Failsafe,
}

pub struct TankAutopilot {
    heading_pid: Pid<F>,
    last_mode: Option<Mode>,
}

impl TankAutopilot {
    pub fn new(pid: (F, F, F)) -> Self {
        Self {
            heading_pid: Pid::<F>::with_custom_error(pid, 0.0, Some((-1.0, 1.0)), angle_error_fn),
            last_mode: None,
        }
    }
}

impl Autopilot for TankAutopilot {
    type Frame = TankOutputFrame;

    fn output_frame(&mut self, input_frame: InputFrame) -> TankOutputFrame {
        let mode = {
            // There are three levels of arming: software (this program is running), rc controlled
            // and autonomous
            const TRIGGER_A: usize = 4;
            const TRIGGER_B: usize = 5;

            match (&input_frame.armed, &input_frame.rc_channels) {
                (Some(armed), Some(rc_channels)) => {
                    if rc_channels.failsafe {
                        Mode::Failsafe
                    } else {
                        if *armed && rc_channels.channels[TRIGGER_A] > 0. {
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

fn angle_error_fn(input: F, target: F) -> F {
    let a = input.rem_euclid(360.) as u32;
    let b = target.rem_euclid(360.) as u32;
    let directed_angle = directed_angle_deg(a, b);
    directed_angle as F / 180.
}



pub struct TankDispatcher {
    pub motors_sender: Sender<(f64, f64)>,
}

impl Dispatcher<TankOutputFrame> for TankDispatcher {
    fn dispatch(&self, output_frame: TankOutputFrame) -> anyhow::Result<()> {
        let motors = (
            output_frame.left_motor as f64,
            output_frame.right_motor as f64,
        );
        self.motors_sender
            .send(motors)
            .map_err(|e| anyhow!("{}", e))
    }
}

pub struct TankCollector {
    input_frame: InputFrame,
}

impl TankCollector {
    pub fn new() -> Self {
        Self {
            input_frame: InputFrame::default(),
        }
    }
}

impl Collector for TankCollector {
    fn collect(&mut self, input: Input) -> InputFrame {
        match input {
            Input::RcChannels(rc_channels) => self.input_frame.rc_channels = Some(rc_channels),
            Input::Armed(armed) => self.input_frame.armed = Some(armed),
            Input::SystemInformation(system_information) => {
                self.input_frame.system_information = Some(system_information)
            }
            Input::OrientationEuler(orientation) => {
                self.input_frame.orientation = Some((orientation, Instant::now()))
            }
        }

        self.input_frame.clone()
    }
}
