# Autopilot

## Architecture

This autopilot software is not implemented in a single control loop, but as multiple specialized threads running 
concurrently forming a pipeline.

Threads are divided into 5 successive layers, that can seen as a phases of a traditional control loop:
- Input controllers (many), responsible for acquiring data from physical devices
- Input collector (only one), responsible for buffering the data sent asynchronously by the input controllers 
and send them to the flight controller
- Autopilot (only one), responsible for computing output values
- Output dispatcher (only one), that send each entry of the global output produced by the autopilot to the 
correct output controller
- Output controllers (many) applying commands to the physical devices

## Features

- [X] Asynchronous logging on local file system & optional telemetry over SSH
- [X] Dual DC motor control via L298N using hardware PWM
- [X] Servo control over hardware PWM
- [X] ESC control over hardware PWM
- [X] BNO055 IMU over software I2C
- [X] MPL3115A2 barometer over software I2C
- [X] PID implementation & tests
- [X] External file configuration
- [ ] GPS over mini-UART (or I2C)


## Building

Cross-compilation is recommended. It requires having installed correct Rust targets 
 (`arm-unknown-linux-gnueabihf` or `armv7-unknown-linux-gnueabihf`), and a valid GCC toolchain (see 
 [Raspberry Pi tools](https://github.com/raspberrypi/tools)). Cargo can then be run either by passing args or by having 
 the following configuration file (`.cargo/config.toml`), which sets the default target and the linker:

- Raspberry Pi 1 & Zero (ARMv6)
```toml
[build]

target = "arm-unknown-linux-gnueabihf"

[target.arm-unknown-linux-gnueabihf]
linker = "arm-unknown-linux-gnueabihf"
rustflags = ["-C", "linker=arm-linux-gnueabihf-gcc"]
```

- Raspberry Pi 3 & 4 (ARMv7)
```toml
[build]

target = "armv7-unknown-linux-gnueabihf"

[target.armv7-unknown-linux-gnueabihf]
linker = "arm-none-linux-gnueabihf-gcc"
rustflags = ["-C", "linker=arm-none-linux-gnueabihf-gcc"]
```