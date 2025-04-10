# Sawyer Robot Simulation and Control

This repository contains a MATLAB/Simulink project for modeling, simulating, and controlling the Sawyer robot, a 7-DOF robotic manipulator. The project is divided into four parts: Kinematics, Dynamics, Inverse Dynamics, and Control. Each part builds on the previous one to create a comprehensive simulation and control framework for the Sawyer robot.

## Project Overview

The Sawyer robot is a 7-DOF robotic manipulator developed by Rethink Robotics. This project aims to model its kinematics and dynamics, compute the inverse dynamics, and design a controller to stabilize the robot at a desired joint configuration. The simulation is implemented in MATLAB and Simulink, using a modular approach with separate functions for each component.

### Parts of the Project

1. **Part 1: Kinematics**
   - **Objective**: Model the forward kinematics of the Sawyer robot to compute the end-effector position and joint positions in Cartesian coordinates.
   - **Implementation**:
     - Used Denavit-Hartenberg (DH) parameters to define the robot's geometry.
     - Implemented in `Frame_origin.m`, which computes the end-effector position (`x`, `y`, `z`) and joint positions based on joint angles `q`.
   - **Output**: End-effector position and joint positions for a given joint configuration.

2. **Part 2: Dynamics**
   - **Objective**: Model the dynamics of the Sawyer robot to simulate its motion under gravity and applied torques.
   - **Implementation**:
     - Computed the mass matrix `M(q)` using `MassMatrix.m`.
     - Computed the nonlinear terms `N(q, qd)` (Coriolis, centrifugal, and gravity effects) using `NEquation.m` with the Newton-Euler recursive algorithm.
     - Simulated the dynamics in Simulink (`Simulation.slx`) using `Dynamic_model.m` to solve:
       \[
       M(q) \ddot{q} + N(q, \dot{q}) = \tau
       \]
     - Initially, `tau = 0` (open-loop simulation) to observe the robot's motion under gravity.
   - **Output**: Joint positions (`q`), velocities (`qd`), and end-effector position over time.

3. **Part 3: Inverse Dynamics**
   - **Objective**: Compute the torques required to achieve a desired motion, effectively solving the inverse dynamics problem.
   - **Implementation**:
     - The inverse dynamics are inherently computed in `Dynamic_model.m` by solving for `tau` given a desired `qdd`.
     - In the context of this project, inverse dynamics are used as part of the control design in Part 4.
   - **Output**: Torques (`tau`) required for a given motion.

4. **Part 4: Control**
   - **Objective**: Design a controller to stabilize the Sawyer robot at a desired joint configuration.
   - **Implementation**:
     - Used a **Computed Torque Control** law with PD feedback.
     - Desired joint angles: `q_r(1) = pi/2`, `q_r(2) = -pi/2`, `q_r(3:7) = pi/4`, with zero velocity and acceleration.
     - Implemented in `Controller.m` with gains `Kp = 100 * eye(7)`, `Kd = 100 * eye(7)`.
     - Simulated in Simulink (`Control.slx`), incorporating the dynamics model and forward kinematics.
   - **Output**: Joint positions converging to the desired configuration, end-effector position, and control torques.

## Repository Structure

- **Simulink Models**:
  - `Simulation.slx`: Simulink model for the dynamics simulation (Part 2).
  - `fcn.slx`: Submodel for defining the desired joint angles (Part 4).
  - `Control.slx`: Simulink model for the control simulation (Part 4).

- **MATLAB Functions**:
  - `Frame_origin.m`: Computes the forward kinematics.
  - `MassMatrix.m`: Computes the mass matrix `M(q)`.
  - `NEquation.m`: Computes the nonlinear terms `N(q, qd)`.
  - `Dynamic_model.m`: Computes the state derivatives for the dynamics simulation.
  - `Applied_Torque.m`: Defines the input torques (used in Part 2, set to zero).
  - `Controller.m`: Implements the control law (Part 4).
  - `fcn.m` (Joint Angle Specification): Defines the desired joint angles.
  - `fcn.m` (Torques1): Extracts `q` and `qd` from the state vector.

- **Documentation**:
  - `Report_Part1.md`: Documentation for the kinematics part.
  - `Report_Part2.md`: Documentation for the dynamics part.
  - `Report_Part3.md`: Documentation for the inverse dynamics part.
  - `Report_Part4.md`: Documentation for the control part.

## Prerequisites

- MATLAB (with Simulink) installed.
- Basic understanding of robotic manipulators, dynamics, and control theory.
