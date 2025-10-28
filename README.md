Simulink Control for Thrust-Vectored Inverted Pendulum

This repository is for the design and simulation of a controller to balance a thrust-vectored inverted pendulum (a "broom balancer") using Simulink.

Project Overview

The goal of this project is to model the highly unstable, non-linear dynamics of a cylinder (like a long pole) and stabilize it in mid-air using a single, gimbaled propeller attached to its base.

The system must actively control its orientation to prevent falling over while simultaneously supporting its own weight.

 The Controller (The Brain)

To stabilize the unstable plant, a cascaded PID control architecture is used. This is a standard in aerospace for robust attitude control.

Outer Loop (Attitude Control): A Proportional (P) controller looks at the angle error (e.g., "we are tilted 2 degrees"). Its output is a desired angular rate (e.g., "we must rotate back at 0.5 rad/s").

Inner Loop (Rate Control): A full PID controller looks at the rate error (Desired Rate vs. Actual Rate). Its output is the physical gimbal angle (alpha) command required to achieve that rate.

This cascade is implemented independently for both the Pitch and Roll axes.
