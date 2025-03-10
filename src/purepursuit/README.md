# Pure Pursuit Algorithm for Autonomous Bike

This repository contains the implementation of the **Pure Pursuit** algorithm for the autonomous bike. The Pure Pursuit algorithm is a path-following method used to guide a robot along a predefined path or trajectory. This implementation integrates with **ROS 2** (Robot Operating System) to handle robot positioning and velocity commands, and it can be extended for controlling motors via the **I2C/SPI/Docker/ROS** protocol.

## Features

- Implements the **Pure Pursuit** path-following algorithm.
- Uses **ROS 2** to subscribe to robot pose and velocity data.
- Publishes calculated velocity commands (linear and angular velocities) to control the robot.
- Can be integrated with hardware via **I2C or SPI/Docker/ROS?** for low-level motor control.
- Utilizes a **lookahead distance** to select the next point along the path for steering.
- Continuously updates the robot's velocity based on its current position and heading.

## Requirements

- **ROS 2** (Humble or later)
- **Python 3.10+**
- `geometry_msgs`, `nav_msgs`, `tf2_ros`, `tf2_geometry_msgs`, and `transforms3d` ROS 2 packages
- `spidev` for SPI communication
- A **robot platform** (or simulator) for receiving pose and velocity data
- **I2C/SPI/Docker/ROS** communication (if interfacing with external hardware)

## Installation

To get started with this project, clone the repository and install the dependencies.

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/purepursuit.git
   cd purepursuit
