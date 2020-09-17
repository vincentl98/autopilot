# Autopilot

## Architecture

The autopilot software is not implemented in a control loop, but rather as multiple specialized threads running 
concurrently. Threads are divided into 5 successive layers, that one can see as a phases of a traditional control
loop:
- Input controllers (many), that are responsible for acquiring data from physical devices (including filtering)
- Input collector (only one), which is responsible for buffering the data sent asynchronously by the input controllers 
and send them to the flight controller
- Flight controller (only one)
- Output dispatcher (only one), that send each entry of the global output produced by the flight controller to the 
correct output controller
- Output controllers (many) applying commands to the physical devices.

## Error handling

No thread should ever panic, as long as the other threads are alive.

## Roadmap

### RC tank (dual motor car with no servo)

- [X] Raspberry Pi 3 cross-compilation setup
- [X] Telemetry over SSH
- [X] System health monitoring
- [X] RC control via SBUS over serial
- [X] Hardware PWM servo control _(will not be used)_
- [X] ESC control over hardware PWM _(will not be used)_
- [X] L298N controller (2 brushed motors control)
- [X] BNO055 controller (heading only)
- [X] PID controller
- [ ] PID tests
- [X] Heading control
- [X] On-disk async logging
- [X] Configuration file for PID values and log level
- [ ] Raspberry Pi Zero cross-compilation setup
- [ ] GPS I2C driver
- [ ] Waypoint mission with GPS
- [ ] Waypoint mission tests
- [ ] 3-axis attitude follow

The limitation with the RC tank is the inertia of the motors, and their lack of precision. For a given input, 
the delivered power can vary depending on the ground and the initial velocity of the vehicle. However, I manage 
to have a working tank that could follow a magnetic heading with +/- 5 degrees accuracy.
 
## Building

Building via cross-compilation is recommended. Cross-compilation requires having installed correct Rust targets 
 (`arm-unknown-linux-gnueabihf` or `armv7-unknown-linux-gnueabihf`), and a valid GCC toolchain (see 
 [Raspberry Pi tools](https://github.com/raspberrypi/tools)). Cargo can then be run either by passing args or by having 
 this configuration (`.cargo/config.toml`), which sets the default target and the linker:

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