# microslam

An extremely lightweight LIDAR-based SLAM for microcontroller, using a robot vacuum cleaner
as the platform. The goal is to create a system that can run on a microcontroller with limited resources,
while still being able to map and navigate a small environment.

## Structure

The project is structured as follows:

- `firmware/`: Contains the code for the microcontroller.
- `slam/`: Contains the core SLAM algorithms and data structures.

## Development

See [`firmware/README.md`](firmware/README.md) for instructions on how to build and flash to the microcontroller.

If using VS Code, the repository contains a number of configuration files for convenience. Using the provided
`workspace.code-workspace` file to open the workspace is convenient for developing when using the ESP-IDF extension.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
