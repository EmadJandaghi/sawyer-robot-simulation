# Part 4: Control of a 7-DOF Robotic Manipulator

## Overview
This part involves designing and implementing a controller for a 7-DOF robotic manipulator to stabilize it at a desired joint configuration. The dynamics model from Part 3 is used, and a computed torque control law with PD feedback is applied to track the desired joint angles.

## Simulink Model Description
The Simulink model (`Control.slx`) implements the following components:

### Desired Trajectory
- **Block**: `Joint angle specification` (uses `fcn.m`).
- **Outputs**:
  - `q_r`: Desired joint positions (`q_r(1) = pi/2`, `q_r(2) = -pi/2`, `q_r(3:7) = pi/4`).
  - `qd_r`: Desired joint velocities (all zero).
  - `qdd_r`: Desired joint accelerations (all zero).
- **Purpose**: Defines the reference trajectory for stabilization.

### Controller
- **Block**: `Controller` (uses `Controller.m`).
- **Inputs**:
  - `M`: Mass matrix.
  - `N`: Nonlinear terms.
  - `e`: Position error (`q_r - q`).
  - `ed`: Velocity error (`qd_r - qd`).
  - `qdd_r`: Desired joint accelerations.
- **Output**:
  - `tau`: Control torques.
- **Control Law**:
  \[
  \tau = M(q) (\ddot{q}_r + K_p e + K_d \dot{e}) + N(q, \dot{q})
  \]
  Since `qdd_r = 0`, `qd_r = 0`, this simplifies to:
  \[
  \tau = M(q) (K_p (q_r - q) + K_d (0 - \dot{q})) + N(q, \dot{q})
  \]
- **Gains**:
  - `Kp = 100 * eye(7)`.
  - `Kd = 100 * eye(7)`.