# microslam firmware

This is the program that runs on an ESP32-S3 microcontroller. It uses the microslam library and interfaces with the sensors and actuators of the vehicle.

The vehicle is a small robot vacuum (the Neato Robotics D7 Connected Robot Vacuum Cleaner) that provides a serial interface for reading sensor data and controlling motors.

## Prerequisites

- Install the [ESP-IDF toolchain](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/linux-macos-setup.html).
- Install `libbsd-dev` (needed for the host tests).

## Building

Note - if you've set up an alias for sourcing the IDF export.sh script as per the above toolchain instructions, remember to run it before building: `get_idf`.

```bash
idf.py build
```

## Flashing

```bash
idf.py flash
```

or 

```bash
idf.py flash monitor
# ctrl-] to exit
# ctrl-T followed by ctrl-F to build and flash
```

## Configuring

See the get started guide linked above for full details.

```bash
idf.py set-target esp32s3
idf.py menuconfig
```
