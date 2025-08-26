# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

This is a CMake-based Raspberry Pi Pico project using micro-ROS:

```bash
# Build the project
mkdir -p build && cd build
cmake ..
make -j4

# Clean build
rm -rf build
```

The main executable is `agent-reconnect` which gets built as a `.uf2` file for flashing to the Pico.

## Project Architecture

This is a robotics project that implements a low-level motor controller using micro-ROS on a Raspberry Pi Pico 2. The system interfaces with topic_based_ros2_control to provide hardware abstraction for differential drive robots.

### Core Components

- **Main Application** (`agent-reconnect.cpp`): Contains the complete motor controller implementation including:
  - MotorController class for direct motor control
  - Direct PWM motor control based on joint velocity commands
  - Encoder-based joint position feedback
  - ROS 2 subscription to `/robot_joint_commands` (sensor_msgs/JointState)
  - ROS 2 publisher for joint states (`/robot_joint_states`)

- **Transport Layer** (`pico_uart_transport.c`, `pico_uart_transports.h`): Custom UART transport implementation for micro-ROS communication

- **Connection Management** (`main.cpp`): Implements agent reconnection logic with state machine for handling micro-ROS agent connectivity

- **Micro-ROS Library** (`libmicroros/`): Pre-built micro-ROS library with comprehensive ROS 2 message support

### Key Features

- **topic_based_ros2_control Integration**: Compatible with ros2_control hardware interface
- **Direct Motor Control**: PWM-based motor control without PID feedback loops
- **Joint Position Feedback**: Encoder-based joint position reporting in radians
- **Joint Velocity Commands**: Direct velocity command processing from ros2_control
- **Agent Reconnection**: Intelligent state machine handles connection/disconnection with automatic retry
- **Time Synchronization**: ROS time sync with the micro-ROS agent

### Hardware Configuration

The code is configured for:
- Raspberry Pi Pico 2 (`PICO_BOARD pico2`)
- UART communication with micro-ROS agent
- PWM motor control on specified GPIO pins
- Quadrature encoders on interrupt-capable pins
- LED status indication

### Message Types

Primary ROS 2 message interfaces for topic_based_ros2_control:
- `sensor_msgs/JointState` (subscription on `/robot_joint_commands`)
- `sensor_msgs/JointState` (publication on `/robot_joint_states`)
- Joint names: `left_wheel_joint`, `right_wheel_joint`

### Development Notes

- The project is implemented in C++ for better ROS 2 integration
- Motor control uses direct PWM without PID - higher-level control happens in ros2_control
- Encoder tick counts and wheel parameters need calibration for specific hardware
- UART transport is the primary communication method with the micro-ROS agent  
- Scale factors for velocity-to-PWM conversion may need tuning per robot
- State machine continuously monitors agent connection with automatic recovery
- LED provides visual feedback: blinking = waiting for agent, solid = connected, off = disconnected
- Motors automatically stop when connection is lost for safety