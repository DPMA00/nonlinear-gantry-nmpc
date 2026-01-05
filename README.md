# Gantry-Systems

## Overview
This repository contains a first-principles nonlinear dynamic model and nonlinear model predictive control implementation for gantry crane systems with suspended loads.

The primary objective is point-to-point motion of the payload without residual oscillations.

- The 2D gantry system is fully modeled, controlled and simulated.
- The 3D gantry system is under active development.

The goal is for the payload to move from point to point without any residual oscillation.

## Demos

### PID Control
2 PID Controllers running at 100 Hz struggle because of the coupled dynamics, showing noticeable oscillations and a relatively long settling time before the payload comes to rest.
<p align="center">
  <img src="https://github.com/user-attachments/assets/e4937f1e-8106-410f-b66c-04ee3dff351b" width="45%" />
  <img src="https://github.com/user-attachments/assets/ef1da5bf-b3af-4718-8338-0e657e0248cf" width="43%" />
</p>

### MPC Control
A single MPC controller running at 10 Hz is able to bring the payload to rest significantly faster and with minimal oscillatory motion by explicitly accounting for the coupled system dynamics and input constraints during the optimization process.
<p align="center">
  <img src="https://github.com/user-attachments/assets/784aca73-190b-4109-941d-1b00bfdc9481" width="45%" />
  <img src="https://github.com/user-attachments/assets/57f43d13-0b8a-4c4c-8678-e41371679b51" width="43%" />
</p>


## 3D Gantry System (Work in Progress)

The modeling of the 3D gantry system is still under development. Currently two modeling approaches are being considered:

* Extension of the current analytical Euler-Lagrange framework
* Modeling via rigid-body dynamics libraries (e.g. [Pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/index.html))
