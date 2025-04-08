# Report: Part 1 - Kinematics and Updating

## Overview
This section details the kinematic analysis and geometric/inertia updates for a 7-DOF Sawyer Robot. The primary objectives are:
- Compute the end-effector position and orientation using forward kinematics.
- Update the robot’s geometric (center of mass) and inertial properties from a global frame to a non-standard frame attached to each link.

## Kinematics
### Objective
Determine the end-effector pose (position and orientation) based on joint angles using the Denavit-Hartenberg (DH) convention.

### Methodology
- **DH Parameters**: The robot is modeled with 7 links, defined by:
  - `d`: Link offsets (meters).
  - `a`: Link lengths (meters).
  - `alpha`: Twist angles (radians).
  - `theta`: Joint angles (input as `Teta`).
- **Transformation Matrices**: 
  - Homogeneous transformation matrices are calculated for each joint relative to the previous frame.
  - Cumulative transformations are computed relative to the base frame.
- **End-Effector Pose**:
  - **Position**: Extracted from the translation component of the final transformation matrix.
  - **Orientation**: Computed as Euler angles (alpha, beta, gamma) from the rotation matrix, with special handling for degenerate cases (β = ±π/2).

### Implementation
- **File**: `Direct_Kinematic.m`
- **Input**: `Teta` (7x1 vector of joint angles in radians).
- **Output**: `pos` (end-effector position [x, y, z] in meters).
- **Simulation**: A Simulink model in `Kinematics_Folder/` visualizes the robot’s motion over time based on joint angle inputs (user-provided file: `Simulink_Model.slx`).

### Key Code Snippet
```matlab
% Compute transformation matrix for each joint
for i = 1:nL
    Ti_im{i} = [cos(Teta(i)), -sin(Teta(i)), 0, a0(i);
                sin(Teta(i))*cos(alpha0(i)), cos(Teta(i))*cos(alpha0(i)), -sin(alpha0(i)), -sin(alpha0(i))*d0(i);
                sin(Teta(i))*sin(alpha0(i)), cos(Teta(i))*sin(alpha0(i)), cos(alpha0(i)), cos(alpha0(i))*d0(i);
                0, 0, 0, 1];
end