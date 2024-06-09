# Datalogger Firmware

This is the firmware that runs on the Microcontroller that does the actual
data logging and communicates with the host computer.

## Building
Each file in the `src/bin/` directory is a separate binary that can be built
and run on the target hardware. For instance
```bash
cargo run --bin bme280 --release
```
will build the `bme280` binary in release mode and attempt to flash it to the
target hardware. Each of these is used to initially create a "driver" for the
specific piece of hardware that is being interfaced with. In the case of the
BME280, the `bme280` binary is used to create a driver that can read temperature, pressure, and humidity values from the sensor.
