# Franka Teleop C++

This repository contains a C++ teleoperation framework for Franka Emika Panda robots, with separate leader and follower executables connected over UDP using Cap'n Proto messages. It also includes a Python ProMP (Probabilistic Movement Primitives) workspace for motion generation and trajectory learning.

## What this project does

- Runs a leader-side robot controller that publishes robot state.
- Runs a follower-side controller that subscribes to the leader state and mirrors the motion.
- Uses YAML-based configuration for robot IPs, ports, gains, and safety settings.
- Includes a Python-based ProMP implementation for learning and replaying motion primitives.

## Repository layout

- [CMakeLists.txt](CMakeLists.txt) — CMake build definition for the C++ components.
- [leader.cpp](leader.cpp) — Leader-side teleoperation node.
- [follower.cpp](follower.cpp) — Follower-side teleoperation node.
- [teleop_config.yml](teleop_config.yml) — Runtime configuration for networking and controller parameters.
- [messages/robot-state.capnp](messages/robot-state.capnp) — Cap'n Proto schema for robot-state messages.
- [ProMP/](ProMP/) — Python scripts related to ProMP-based motion learning.
- [utils/](utils/) — Shared helper code for robot examples and velocity observation.

## Requirements

Before building, make sure the following are installed:

- CMake
- C++20-compatible compiler
- Cap'n Proto
- Eigen3
- yaml-cpp
- libfranka

The current build file expects libfranka to be available in a custom location. If your installation differs, update the path in [CMakeLists.txt](CMakeLists.txt) accordingly.

## Build

From the repository root:

```bash
cmake -S . -B build
cmake --build build -j
```

This will build the executables for the leader and follower nodes.

## Configuration

Edit [teleop_config.yml](teleop_config.yml) to set:

- the robot IP addresses
- UDP ports
- control gains
- gripper settings
- sampling frequency and safety thresholds

## Run

After building, run the executables from the build directory:

```bash
./build/leader
./build/follower
```

Press `q` in the terminal to stop the running process.

## ProMP scripts

The Python code in [ProMP/Full_ProMP.py](ProMP/Full_ProMP.py) provides a starting point for ProMP-based trajectory modeling and learning. It can be used alongside the C++ teleop stack for demonstration and motion generation experiments.

## Notes

- This project is intended for research and robotics experimentation.
- Ensure your robot network configuration is correct before running the controllers.
- The code assumes a working Franka robot setup and proper permissions for robot access.
