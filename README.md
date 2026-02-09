Micromouse Developer Reference (v2.0)
=====================================

1. System Architecture Overview
-------------------------------
This robot uses a hierarchical control system split into three layers:
1.  High-Level Solver (solver.cpp): Maintains the maze state and determines the next cardinal direction (North, South, East, West).
2.  Mid-Level Navigator (withGyroscope.ino): Converts cardinal moves into physical maneuvers (Turn 90, Straight 1 cell) and manages the robot's heading.
3.  Low-Level Actuator (motor.cpp): Controls wheel velocity using feedforward and feedback loops.

2. Control System Implementation
--------------------------------
The robot uses a Hybrid Control Strategy to overcome mechanical stiction and sensor limitations, deviating from standard PID theory.

A. The Turning Controller (Hybrid Min-Speed + PID)
The robot does not rely solely on PID for turning. Instead, it uses a Trapezoidal-like profile enforced by logic clamps.
* The Problem: Pure PID control with small errors (e.g., 90 deg turn) generates low voltage, failing to overcome gearbox static friction.
* The Solution: A "Speed Floor" forces the motors to move at a minimum speed until the robot is very close to the target.
* Logic Flow:
    1.  Cruise Phase (Error > 15 deg): The controller overrides the PID output and forces 'min_turn_speed' (3.0 rad/s). This ensures consistent rotation regardless of the target angle.
    2.  Landing Phase (Error < 15 deg): The logic hands control back to the PID controller to gently guide the robot to the stop point.
    3.  Stop Threshold: Control cuts off completely when error < 0.5 deg to prevent oscillation.

B. Gyroscope Integration & Drift Compensation
The robot tracks its absolute heading ('position') by integrating the Z-axis gyroscope data.
* Drift Correction (Scalar): A scalar value of 0.9935 is applied to the integration step to correct for sensor scale factor error.
* Drift Correction (Heading Hold): When driving straight, a secondary PID loop ('drift_controller') compares the current 'position' against a locked 'baseline_position'. It injects a differential speed adjustment to the motors to force the robot back to the centerline.

C. Sensor Configuration
* Range: The BMI270 gyroscope is configured for 1000 dps (Degrees Per Second). Note: While 2000 dps is preferred for high-speed spins, the current codebase uses 1000 dps.
* Filtering: The Derivative (D) term of the PID controller utilizes a Low-Pass Filter (Bilinear Transform) to reject noise. The cutoff frequency is currently set to 1.0 Hz.

3. Motor Driver Implementation
------------------------------
The Motor class implements a velocity control loop that runs at 200Hz (every 5ms).

Control Equation:
PWM = (K_ff * V_target) + PID(V_error)

* Feedforward: Calculates theoretical PWM needed for target velocity using 'feedforward_gain_v'.
* Feedback: The 'velocity_controller' compensates for battery voltage drop and friction.
* Deadband Handling: The controller zeros out PWM if the target velocity is effectively zero to prevent motor hum.

4. Maze Solver Logic (solver.cpp)
---------------------------------
The robot uses a Floodfill Algorithm to navigate.

A. Coordinate System & Wall Detection
* Orientation Tracking: The solver infers the robot's cardinal direction based on velocity variables (bot_x_velocity, bot_y_velocity).
* Wall Updates: Walls are detected relative to the robot (Front/Left/Right) using IR sensors and mapped to absolute grid coordinates.

B. Decision Logic (decideBestMove)
The robot evaluates neighbors in the order: Left -> Right -> Front.
1.  Checks validity (no wall, within bounds).
2.  Compares "Floodfill Distance" (steps to goal).
3.  Prioritizes the neighbor with the lowest distance value.

C. State Machine
* Init Phase: Grid initialized with goal at center (4 cells).
* Search Phase: Explores until center is reached (Distance == 0).
* Return Phase: Goal coordinates swapped to Start Cell (0,0) to return home.

5. Key Tuning Constants
-----------------------
* Gyro Scale (withGyroscope.ino): 0.9935f - Multiplier to fix sensor scale error.
* Speed Floor (withGyroscope.ino): 3.0f - Minimum turning speed when error > 15 deg.
* Turn PID (withGyroscope.ino): Kp=7e-3, Ki=1e-3, Kd=1e-2.
* Drift PID (withGyroscope.ino): Kp=2.5, Ki=0.5, Kd=0.15.
