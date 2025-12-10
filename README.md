# E-Rocket Flight Software

This repository contains the code and documentation for the E-Rocket flight software. 

## Table of Contents

- [Project Overview](#project-overview)
- [Getting Started](#getting-started)
- [Repository Structure](#repository-structure)

## Project Overview

The E-Rocket project aims to develop an electric-powered, low-cost rocket prototype for validation of Guidance, Navigation & Control algorithms based on Thrust Vector Control.
In terms of software, it combines PX4 autopilot with ROS 2 middleware. 

## Getting Started

Pre-requisites:

1. Install ROS 2 (The team used Jazzy Jalisco) using the [official ROS 2 installation guide](https://docs.ros.org/en/jazzy/Installation.html).
2. Download the [E-Rocket repository](https://github.com/PedromcaMartins/e-rocket).
3. Download [PX4 Autopilot fork with modified firmware](https://github.com/PedromcaMartins/PX4-Autopilot/tree/e-rocket). Use branch e-rocket.
4. Follow PX4 Autopilot tutorial on [ROS 2 instalation](https://docs.px4.io/main/en/ros2/user_guide).
5. (Optional) Install [`mprocs`](https://github.com/pvolok/mprocs). This tool allows to easily run multiple processes in different terminal tabs. Used for launching and using the ROS 2 nodes.
6. (Optional) Install [`Plot Juggler`](https://docs.ros.org/en/jazzy/p/plotjuggler/).

To build and run the project:

1. Run `mprocs`:
```bash
cd workspace_erocket/src
mprocs
```

2. On the second tab, run the process called `colcon build`

3. Launch the ROS 2 nodes:

- `Offboard_Baseline`: Launches required nodes for flight testing with baseline offboard control (PID Controller).

- `Offboard_Generic`: Launches required nodes for flight testing with generic offboard control. This controller is to be implemented by the user.

- `SITL_SIMULINK_*`: Launches the closed source Simulink model for Software In The Loop (SITL) simulation. Different versions are available depending on the controller used (Baseline or Generic), but none are yet available to the public.

## Repository Structure

The repository is structured as follows:

```
E-Rocket
├───Documentation           # Project documentation
├───PX4 Params              # PX4 parameter files for indoor and outdoor flights
└───workspace_erocket       # ROS 2 Workspace
    │
    ├───rosbags             # Recorded ROS 2 bag files from test flights
    ├───trajectory          # Scripts to generate trajectories for flight tests
    └───src                 # ROS 2 packages
        │
        ├───erocket         # E-Rocket specific ROS 2 nodes and configurations
        ├───mocap_interface # Motion capture system interface that subscribes to VRPN topics and republishes to PX4
        ├───px4_msgs        # PX4 message definitions for ROS 2 - From official repository
        ├───px4_ros_demos   # PX4 ROS 2 demo nodes - From official repository
        └───vrpn_vendor     # VRPN client library for ROS 2
```

- `mocap_interface` & `mocap_interface`: These packages were adapted from the [Pegasus project](https://github.com/PegasusResearch/pegasus).
- `px4_msgs`: This package was sourced from [official Repository](https://github.com/PX4/px4_msgs).
- `px4_msgs`: This package was sourced from [official Repository](https://github.com/PX4/px4_ros_com).