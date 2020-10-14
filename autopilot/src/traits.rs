use crate::input::Input;
use crossbeam_channel::{Receiver, Sender};
use std::{
    error::Error,
    fmt::Display,
    thread,
    time::{Duration, Instant},
};

pub trait Autopilot<In, Out>
where
    Self: Sized + Send + Sync + 'static,
    Out: Send + 'static,
    In: Send + 'static,
{
    const MAX_CONTROL_LOOP_PERIOD: Duration;

    fn output_frame(&mut self, input_frame: In) -> Out;

    fn control_loop(&mut self, receiver: Receiver<In>, sender: Sender<Out>) {
        let mut last_output_frame_instant = Instant::now();

        // Empty the receiver queue, while keeping the latest input frame.
        // Starvation is avoided by setting a maximum period in which an output frame should be
        // emitted.
        while let Ok(input_frame) = receiver.recv() {
            if receiver.is_empty()
                || (Instant::now() - last_output_frame_instant) >= Self::MAX_CONTROL_LOOP_PERIOD
            {
                sender.send(self.output_frame(input_frame)).unwrap();

                last_output_frame_instant = Instant::now();
            }
        }
    }

    fn spawn(mut self, receiver: Receiver<In>, sender: Sender<Out>) -> thread::JoinHandle<()> {
        thread::spawn(move || self.control_loop(receiver, sender))
    }
}

/// Controllers that filters and buffers asynchronous inputs
pub trait Collector<F: Send + 'static>
where
    Self: Sized + Send + 'static,
{
    fn collect(&mut self, input: Input) -> F;

    fn collect_loop(&mut self, receiver: Receiver<Input>, sender: Sender<F>) -> ! {
        for input in receiver {
            sender.send(self.collect(input)).unwrap();
        }

        unreachable!()
    }

    fn spawn(mut self, receiver: Receiver<Input>, sender: Sender<F>) -> thread::JoinHandle<()> {
        thread::spawn(move || self.collect_loop(receiver, sender))
    }
}

/// Controllers that dispense outputs
pub trait Dispatcher<F: Send + Sized + 'static>
where
    Self: Sized + Send + 'static,
{
    fn dispatch(&self, frame: F);

    fn dispatch_loop(&self, receiver: Receiver<F>) -> ! {
        for frame in receiver {
            self.dispatch(frame);
        }

        unreachable!()
    }

    fn spawn(self, receiver: Receiver<F>) -> thread::JoinHandle<()> {
        thread::spawn(move || self.dispatch_loop(receiver))
    }
}

/// Controllers that import external data.
pub trait InputController
where
    Self: Sized + Send + 'static,
{
    /// Minimum duration to wait between two successive `read_input` calls.
    const DELAY: Option<Duration>;

    fn read_input(&mut self) -> Result<Input, Box<dyn Error>>;

    fn read_loop(&mut self, input_sender: Sender<Input>) -> ! {
        loop {
            match self.read_input() {
                Ok(input) => input_sender.send(input).unwrap(),
                Err(e) => error!("{}", e),
            }

            if let Some(delay) = Self::DELAY {
                thread::sleep(delay);
            }
        }
    }

    /// Spawns a thread running `read_loop`. It is expected that the input controller is ready to
    /// read when spawned, i.e. that the initialization (if any) is done.
    fn spawn(mut self, input_sender: Sender<Input>) -> thread::JoinHandle<()> {
        thread::spawn(move || self.read_loop(input_sender))
    }
}

/// Controllers that logs internal data.
pub trait Monitor<T: Display>
where
    Self: Sized + Send + 'static,
{
    /// Minimum duration to wait between two successive `monitor` calls.
    const DELAY: Option<Duration>;

    fn monitor(&mut self) -> Result<T, Box<dyn Error>>;

    fn monitor_loop(&mut self) -> ! {
        loop {
            match self.monitor() {
                Ok(data) => info!("{}", data),
                Err(e) => error!("{}", e),
            }

            if let Some(delay) = Self::DELAY {
                thread::sleep(delay);
            }
        }
    }

    fn spawn(mut self) -> thread::JoinHandle<()> {
        thread::spawn(move || self.monitor_loop())
    }
}

/// Controllers that export data to external devices.
pub trait OutputController<T: Send + 'static>
where
    Self: Sized + Send + 'static,
{
    fn write_output(&mut self, output: T) -> Result<(), Box<dyn Error>>;

    fn write_loop(&mut self, output_receiver: Receiver<T>) -> ! {
        loop {
            for output in output_receiver.iter() {
                self.write_output(output)
                    .map_err(|e| error!("{}", e))
                    .unwrap_or_default();
            }

            unreachable!()
        }
    }

    fn spawn(mut self, output_receiver: Receiver<T>) -> thread::JoinHandle<()> {
        thread::spawn(move || self.write_loop(output_receiver))
    }
}
