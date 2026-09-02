# 🦾 SCARA Assembly: Kinematics & Dynamic Movement Primitives

An end-to-end implementation of an automated robotic assembly task using a SCARA arm, developed for the course *Robotics* (2025–2026) at the Aristotle University of Thessaloniki, Department of Electrical and Computer Engineering.

The project is built in **two parts**, progressing from ideal trajectory planning to adaptive, real-time control using Dynamic Movement Primitives (DMPs).

---

## 📋 Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Part 1 — Ideal Trajectory & Kinematics](#part-1--ideal-trajectory--kinematics)
- [Part 2 — Dynamic Movement Primitives (DMPs)](#part-2--dynamic-movement-primitives-dmps)
- [Results & Key Highlights](#results--key-highlights)
- [Installation](#installation)
- [Usage](#usage)

---

## Overview

This repository simulates a factory setup where a **SCARA robot** must synchronize with moving conveyors to perform a pick-and-place and assembly operation:
- **Conveyor A:** Moves continuously, carrying *Part A*.
- **Conveyor B:** Moves in discrete steps, carrying *Part B* (the socket).
- **Task:** The robot intercepts Part A on the fly, transports it over Part B, and performs an exact 90° screwing motion to assemble them.

---

## Project Structure

```text
.
├── code/                        # MATLAB source files
│   ├── animate_scene.m          # Custom 3D rendering engine
│   ├── create_robot.m           # SCARA DH-parameters setup
│   ├── key_poses.m              # Dynamic waypoint calculator
│   ├── main_part1.m             # Executable for ideal conditions
│   ├── main_part2.m             # Executable for DMP adaptation
│   ├── plan_trajectory.m        # 5th-order polynomial generator
│   ├── scara_ikine.m            # Analytical Inverse Kinematics solver
│   ├── simulate_dmp.m           # DMP integration (supports initial velocity)
│   └── train_dmp.m              # DMP weight training via Least Squares
│
├── figures/                     # Generated plots and simulation graphs
|
├── report.pdf
└── robotics_project26.pdf
```

---

## Part 1 — Ideal Trajectory & Kinematics

Focuses on the baseline operation under perfect environmental conditions.

- **Trajectory Planning:** Generates $C^1$ continuous paths using 5th-order polynomials (`plan_trajectory.m`). Ensures perfect velocity matching with Conveyor A ($v_x = 0.3 \text{ m/s}$) during the grasping phase to eliminate mechanical shock.
- **Analytical Inverse Kinematics:** Implements a custom, highly optimized geometric IK solver (`scara_ikine.m`). It utilizes previous joint states (`q_prev`) to maintain elbow branch continuity and actively prevents $360^\circ$ angle-wrapping anomalies on the end-effector.
- **Collision Avoidance:** Introduces an intermediate clearance pose (`entry_B`) separating the horizontal transit from the vertical insertion, protecting the socket walls from collisions.

---

## Part 2 — Dynamic Movement Primitives (DMPs)

Introduces spatial uncertainty, requiring the robot to adapt to random positional ($\delta_x, \delta_y$) and rotational ($\theta_\delta$) misalignments of Part B on Conveyor B.

- **Non-Zero Initial Velocity Tracking:** Upgrades the standard DMP formulation to accept initial spatial velocities (`dy0`). This prevents the characteristic "jerk" when transitioning from the conveyor-tracking phase into the DMP-driven delivery phase.
- **Task Separation (Scaling vs. Geometry):** Carefully decouples spatial scaling from strict physical constraints. While the position $(x, y, z)$ is fully governed by DMPs, the end-effector orientation aligns with the misaligned base during transit and then strictly executes a pure $+90^\circ$ screw motion, avoiding incorrect rotational scaling.
- **Numerical Stability:** Incorporates zero-division guards during training to handle zero-variance axes (e.g., maintaining constant height during horizontal transit) without producing `NaN` weights.

---

## Results & Key Highlights

The custom implementation successfully resolves standard academic DMP pitfalls, yielding industrial-grade kinematic profiles:

| Feature | Description | Impact |
| :--- | :--- | :--- |
| **Smooth DMP Transition** | Initial velocity injection (`dy0`) into the DMP state variables. | Eliminates acceleration spikes at the hand-off phase ($t=2.5\text{s}$), ensuring safe motor operation. |
| **Strict Screw Logic** | Analytical overwrite of the orientation DMP. | Guarantees exactly $90^\circ$ of rotation for the assembly, regardless of the base's initial angular deviation. |
| **Realistic Scheduling** | Expanded temporal window for the screwing phase (1.0s). | Keeps maximum angular velocity capped at a safe $\approx 170\text{ deg/s}$, avoiding impossible motor strain. |
| **3D Rendering Engine** | Custom patch-based graphics bypassing Robotics Toolbox limits. | Allows dynamic visualization of the opening/closing gripper and moving objects at 40 FPS. |

<div align="center">
  <img src="figures/Pos + Orient DMP - EE orientation.png" width="45%" hspace="2%" />
  <img src="figures/Pos + Orient DMP - Joint velocities.png" width="45%" />
  <p><em>Left: Perfect separation of base alignment and strict 90° assembly rotation. Right: Smooth, jerk-free joint velocities despite dynamic target adaptation.</em></p>
</div>

---

## Installation

### Requirements

- **MATLAB** (Tested on R2023a+)
- **Robotics Toolbox for MATLAB** (by Peter Corke - Version 9 or 10)

Ensure the Robotics Toolbox is added to your MATLAB path before running the scripts.

---

## Usage

Navigate to the `code/` directory and run the main scripts directly from the MATLAB command window or editor.

```matlab
% Run Part 1: Ideal Assembly Cycle (2 continuous cycles)
main_part1

% Run Part 2: DMP Adaptation (Tests 3 different deviation scenarios)
% Note: The simulation will pause between the Position-Only DMP test 
% and the Full Pos+Orientation DMP test. Press ENTER in the Command Window to continue.
main_part2
```
