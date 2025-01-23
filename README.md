# ROS Workspace - Navigation with IMU and Magnetometer

This repository contains a ROS workspace for collecting, processing, and analyzing IMU and Magnetometer data.

## 1. Workspace Structure

The workspace is organized as follows:

- **nav_driver**: The package responsible for handling GPS & IMU data acquisition, implementing a driver that interacts with the IMU & GPS hardware .
- **imu_msgs**: Contains custom ROS message type definitions used for handling IMU data within the workspace.
- **gps_msgs**: Contains custom ROS message type definitions used for handling GPS data within the workspace.


## 2. Running the Nav Driver

To run the Nav driver, use the provided launch file. This file is designed to start the driver nodes that interacts with IMU & GPS.

### Steps to build the workspace:

1. Build the workspace using `colcon`:
   ```bash
   colcon build
   ```
2. Source the `setup.bash` file to configure the environment:
   ```bash
   source install/setup.bash
   ```

### Steps to launch the Nav driver:

1. Run the launch file:
   ```bash
   ros2 launch nav_driver nav_launch.py gps_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1
   ```
   - Replace `/dev/ttyUSB0`, `/dev/ttyUSB1` with the appropriate port connected to the IMU & GPS.
   - The launch file starts the `drivers` executable within the `nav_driver` package, responsible for collecting data from the IMU & GPS and publishing it on relevant topics.
