# Robotic Arm Emulation

This project is an embedded control system for a robotic arm implemented in C on an STM32 microcontroller. The goal is to control arm motion, read sensor feedback, validate motion accuracy, and manage gripper behavior in a structured and organized way.

## Project Overview
The main development work for this project was completed in the `Controller` folder. This folder contains the firmware I implemented for the robotic arm logic, motor control, state management, and feedback handling.

The `Sensor_gei1093_3` folder was provided by the professor and serves as the sensor-side project used alongside the controller implementation. It is not the primary code developed by me, but it is an important part of the full system integration.

## Repository Structure
```text
Robotic_arm_emulation/
├── Controller/
│   ├── Controller.ioc
│   └── Core/
│       ├── Inc/
│       └── Src/
├── Sensor_gei1093_3/
│   ├── Sensor_gei1093_3.ioc
│   └── Core/
├── .gitignore
├── README.md
```

## Main Components
- `Controller/` – my codebase for the robotic arm controller
- `Controller/Core/Src` – main firmware logic, state machine, motion computation, PWM control, and sensor interaction
- `Controller/Core/Inc` – generated driver headers and interface definitions
- `Sensor_gei1093_3/` – provided by the professor for sensor support and system integration

## Features
- State machine logic
- ADC and SPI sensor integration
- PWM servo control
- Position verification and correction
- Gripper activation and object detection
- Emergency stop and reset logic
- Organized embedded firmware structure

## Control Flow
The program follows a state-based workflow:
1. Compute target arm position
2. Send motor commands
3. Wait for servo stabilization
4. Read sensor feedback
5. Verify position accuracy
6. Correct errors if needed
7. Activate gripper
8. Return to the initial state

