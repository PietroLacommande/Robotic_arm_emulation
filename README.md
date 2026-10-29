# Robotic Arm Emulation

A structured embedded control project for a robotic arm emulator built around an STM32 microcontroller. The project focuses on real-time motion control, sensor feedback, PWM actuation, and state-based operational logic.

This repository showcases the controller logic implemented in the `Controller` folder, while the `Sensor_gei1093_3` project was provided by the professor as the sensor-side reference/development environment.

## Project overview

The system is designed to emulate and control a 3-axis robotic arm using:

- PWM signal generation for servo control
- ADC-based signal acquisition for gripper detection
- SPI communication for reading angular feedback
- GPIO-based I/O and emergency stop handling
- Finite-state control logic for compute, send, read, correct, grip, and reset cycles

The main firmware is centered in:

- `Controller/Core/Src/main.c`
- `Controller/Core/Inc/main.h`

## Repository structure

```text
Robotic_arm_emulation/
├── Controller/
│   ├── Controller.ioc
│   └── Core/
│       ├── Inc/
│       │   ├── adc.h
│       │   ├── gpio.h
│       │   ├── main.h
│       │   ├── spi.h
│       │   ├── stm32l4xx_hal_conf.h
│       │   ├── stm32l4xx_it.h
│       │   ├── tim.h
│       │   └── usart.h
│       └── Src/
│           ├── adc.c
│           ├── gpio.c
│           ├── main.c
│           ├── spi.c
│           ├── stm32l4xx_hal_msp.c
│           ├── stm32l4xx_it.c
│           ├── syscalls.c
│           ├── sysmem.c
│           ├── system_stm32l4xx.c
│           ├── tim.c
│           └── usart.c
├── Sensor_gei1093_3/
│   ├── Sensor_gei1093_3.ioc
│   └── Core/
│       ├── Inc/
│       └── Src/
├── .gitignore
├── README.md
└── LICENSE (if present in the project)
```

## Main controller functionality

The controller side implements the central robotic arm behavior in `main.c` and is organized around a state machine.

### Core behaviors

- Position planning using inverse kinematics for target coordinates
- Conversion from geometric arm angles to PWM duty cycle values
- Motor command transmission through timer channels
- Feedback acquisition from sensor data over SPI
- Error detection and correction for base angle mismatch
- Gripper activation and object detection
- Emergency-stop safety mode
- Reset to initial position after each cycle

### State machine

The firmware follows a structured workflow:

1. `STATE_COMPUTE`  
   Calculates the desired target position and the corresponding arm angles.

2. `STATE_SEND`  
   Sends the computed PWM values to the actuators.

3. `STATE_WAIT`  
   Allows the servo system to settle after movement.

4. `STATE_READ`  
   Reads the measured angle values from the sensor.

5. `STATE_VERIFY`  
   Compares measured and desired positions.

6. `STATE_CORRECT`  
   Adjusts incorrect base-angle values.

7. `STATE_GRIP`  
   Activates the gripper and validates object capture.

8. `STATE_RESET`  
   Returns the arm to the initial configuration.

## Hardware and embedded architecture

This project is built for STM32-based development and leverages STM32 HAL drivers generated through STM32CubeMX / `.ioc` configuration.

### Peripheral usage

- `TIM1` for PWM servo control
- `TIM6` for periodic superloop timing
- `ADC1` for gripper sensing / voltage acquisition
- `SPI2` for sensor communication
- `USART2` for serial communication support
- `GPIO` for LED indicators and control signals

## Key technical features

### 1. Kinematic arm control
The controller computes target arm orientation from x/y/z position inputs using geometric relationships and inverse kinematics. The code converts these values into servo command values using a centered CCR conversion model.

### 2. Closed-loop correction
The system reads the actual angular state from the sensor, compares it with the desired position, and applies a correction step if the base angle is off.

### 3. Safety logic
The firmware includes an emergency stop mechanism triggered through an external interrupt. When the stop is active, the controller disables PWM and halts motion.

### 4. Gripper interaction
The project includes a simple gripper activation routine and a reading path through ADC to determine whether an object has been successfully grasped.

## Notes on the sensor project

The folder `Sensor_gei1093_3` is a separate STM32 project provided by the professor for the sensor side of the system. It is not the main implementation area of this repository; rather, it complements the controller code and provides the sensor infrastructure used during development.

This repository is focused on the control-side implementation and the embedded logic that drives the robotic arm behavior.

## What this project demonstrates

This project reflects a strong embedded systems skill set, including:

- Real-time firmware design
- Microcontroller peripheral configuration
- PWM control and actuator synchronization
- Sensor readback and feedback validation
- State-machine software architecture
- Safety mechanisms in embedded systems
- Hardware-software integration in C

## Recommended usage

1. Open the `Controller` project in STM32CubeIDE or equivalent IDE.
2. Configure and build the firmware for the target STM32 board.
3. Ensure the sensor module and actuator wiring match the SPI / PWM / GPIO interfaces.
4. Use the state-machine workflow and calibration logic to tune servo movement.

## Acknowledgements

- `Controller` code was developed as the main project implementation.
- `Sensor_gei1093_3` was provided by the professor as part of the laboratory/school assignment.
- STM32 HAL libraries and Cube-generated files support the embedded application architecture.

## Conclusion

This project demonstrates a practical application of embedded control and robotics principles in C. It combines motion planning, sensor feedback, actuator command handling, and safety-oriented software design into a single, organized controller firmware project.

The code is structured around clear separation between system setup, peripheral initialization, and logic stages, making it a good example of organized embedded software design.
