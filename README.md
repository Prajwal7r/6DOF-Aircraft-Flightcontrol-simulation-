# 6DOF-Aircraft-Flightcontrol-simulation-
Simulates 6DOF nonlinear aircraft dynamics in MATLAB Simulink with control inputs, NED and geodetic position outputs, and real-time 3D animation using the Aircraft Animation Toolbox. Ideal for flight control and dynamics visualization.
Understood. Here's a clean and professional `README.md` without emojis, suitable for your GitHub repository:

---

# 6DOF Aircraft Flight Control Simulation

This project simulates a 6 Degrees of Freedom (6DOF) nonlinear aircraft model using MATLAB Simulink. The model uses control inputs from the elevator, rudder, aileron, and dual-engine throttle to compute full aircraft dynamics.

## Features

* Nonlinear 6DOF aircraft dynamic model
* Outputs: body-frame velocities, Euler angles (roll, pitch, yaw), and angular rates (p, q, r)
* Computes position and velocity in the NED (North-East-Down) frame
* Converts position to geodetic coordinates (latitude, longitude, altitude)
* Real-time aircraft animation using the Aircraft Animation Toolbox

## Repository Contents

* `rcam.m`: Function file defining aircraft dynamics used in the Simulink model
* `initialization_integrator_constant.m`: Initializes simulation parameters and runs the model
* Simulink model file (`.slx`)
* Aircraft Animation Toolbox files (required for visualization)

## How to Run

1. Clone or download the repository.
2. Open MATLAB and run the script `initialization_integrator_constant.m`.
3. This will load all constants, set initial conditions, and start the Simulink model.
4. Ensure `rcam.m` is in your MATLAB path before simulation.

## Initial Conditions

```matlab
x0 = [85; 0; 0; 0; 0; 0; 0; 0; 0];
v0 = [0; 0; 0];
u  = [0; -0.1; 0.05; 0.08; 0.08];  % [aileron; elevator; rudder; throttle1; throttle2]
```

These values can be modified in the script to simulate different flight scenarios.

## License

This project is intended for academic and research use. Please cite or acknowledge appropriately when used.

---

Let me know if you want to add diagrams or usage examples.
