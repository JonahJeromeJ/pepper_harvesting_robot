# 4-DOF Robotic Arm – Kinematics, Vision & Distance Sensing

## Overview

This project involves the design and implementation of a 4-DOF
robotic arm integrating mechanical design, kinematic modelling,
servo control, camera-based coordinate estimation and distance
sensing.

The robotic arm consists of four controllable joints:

- Base
- Shoulder
- Elbow
- Wrist

The software combines Forward Kinematics and numerical Inverse
Kinematics to determine and control the end-effector position.

The project also includes a VL53L0X distance sensor and a
camera-based coordinate estimation system.

## Key Features

- 4-DOF robotic arm
- Forward Kinematics
- Numerical Inverse Kinematics
- Cartesian X-Y-Z position calculation
- Servo motor control using Raspberry Pi GPIO
- VL53L0X distance sensing
- Camera-based coordinate estimation
- SolidWorks mechanical design
- Complete CAD parts and final assembly

## Kinematics

### Forward Kinematics

Forward Kinematics is used to calculate the end-effector position
from the given joint angles.

The model calculates the X, Y and Z coordinates using the
dimensions of the robotic arm and its joint angles.

### Inverse Kinematics

The project uses a numerical search approach for Inverse Kinematics.

Possible joint configurations are generated at 3° intervals.
Forward Kinematics is then used to calculate the end-effector
position for each configuration.

The calculated positions are compared with the desired X-Y-Z
position, and the configuration with the minimum positional error
is selected.

The corresponding joint angles are then sent to the servo motors.

## Distance Sensing

A VL53L0X time-of-flight distance sensor is interfaced through I²C
to obtain distance measurements in millimeters.

## Camera-Based Coordinate Estimation

The camera system uses calibration parameters to convert pixel
coordinates into real-world X-Y-Z coordinates.

## CAD Design

The mechanical components of the robotic arm were designed using
SolidWorks.

The repository includes the individual 3D parts and the final
assembly.

## Hardware

- Raspberry Pi
- 4 Servo Motors
- VL53L0X Distance Sensor
- Camera
- Robotic Arm mechanical components

## Software & Tools

- Python
- NumPy
- OpenCV
- lgpio
- SolidWorks
- I²C

## Project Demonstration

The final robotic arm assembly is demonstrated in the video below.

[Watch the Final Assembly Demonstration](Robot_Arm.gif.gif)
