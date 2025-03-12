# Controlling N link serial planar manipulator

## Project Overview
This project focuses on modeling and controlling a two-link manipulator using system dynamics and control techniques. The system consists of two connected masses, where the goal is to move the joint angles from an initial position to a desired position using different control strategies.

## Control Strategies
We implemented the following control techniques:
- **Proportional (P) Control**: Fast response but causes steady-state error.
- **Proportional-Integral (PI) Control**: Reduces steady-state error but may introduce oscillations.
- **Proportional-Derivative (PD) Control**: Reduces overshoot and improves stability.
- **Proportional-Integral-Derivative (PID) Control**: Provides the best performance with minimal error and fast settling time.

## Implementation
- Simulated using MATLAB with `ode45`.
- Developed Simulink models for different controllers.
- Best tuning parameters: \( K_p = 200, K_d = 100, K_i = 150 \).

## Results
- The **PID controller** performed the best with smooth and stable tracking.
- The **PD controller** improved damping but had steady-state error.
- The **PI controller** eliminated steady-state error but introduced oscillations.
- The **P controller** was unstable for high \( K_p \) values.

