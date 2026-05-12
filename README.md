# Micromouse Developer Reference (v2.0)

## 1. System Architecture Overview
This robot uses a hierarchical control system split into three layers:
1.  **High-Level Solver (`solver.cpp`):** Maintains the maze state and determines the next cardinal direction (North, South, East, West).
2.  **Mid-Level Navigator (`withGyroscope.ino`):** Converts cardinal moves into physical maneuvers (Turn 90°, Straight 1 cell) and manages the robot's heading.
3.  **Low-Level Actuator (`motor.cpp`):** Controls wheel velocity using feedforward and feedback loops.

## 2. Control System Implementation
Unlike standard textbook PID controllers, this robot uses a **Hybrid Control Strategy** to overcome mechanical stiction and sensor limitations.

### A. The Turning Controller (Hybrid Min-Speed + PID)
The robot does not rely solely on PID for turning. Instead, it uses a **Trapezoidal-like profile** enforced by logic clamps.

* **The Problem:** Pure PID control with small errors (e.g., 90° turn) generates low voltage, often failing to overcome gearbox static friction (stiction).
* **The Solution:** A "Speed Floor" forces the motors to move at a minimum speed until the robot is very close to the target.
* **Logic Flow:**
    1.  **Cruise Phase ($Error > 15^{\circ}$):** The controller overrides the PID output and forces `min_turn_speed` (set to 3.0 rad/s). This ensures consistent rotation regardless of the target angle.
    2.  **Landing Phase ($Error < 15^{\circ}$):** The logic hands control back to the PID controller to gently guide the robot to the stop point.
    3.  **Stop Threshold:** Control cuts off completely when error $< 0.5^{\circ}$ to prevent oscillation.

### B. Gyroscope Integration & Drift Compensation
The robot tracks its absolute heading (`position`) by integrating the Z-axis gyroscope data.

* **Drift Correction (Scalar):** A scalar value of `0.9935` is applied to the integration step to correct for sensor scale factor error (where the sensor reads faster/slower than reality). 
* **Drift Correction (Heading Hold):** When driving straight, a secondary PID loop (`drift_controller`) compares the current `position` against a locked `baseline_position`. It injects a differential speed adjustment (`correction`) to the motors to force the robot back to the centerline.

### C. Sensor Configuration
* **Range:** The BMI270 gyroscope is configured for **1000 dps** (Degrees Per Second).
    * *Note:* Turning speeds can exceed this range. Care should be taken to ensure the robot does not saturate the sensor during high-speed spins.
* **Filtering:** The Derivative (D) term of the PID controller utilizes a Low-Pass Filter (Bilinear Transform) to reject noise.
    * *Current Setting:* The cutoff frequency (`fc`) is currently set to **1.0 Hz** in `withGyroscope.ino`.

## 3. Motor Driver Implementation
The `Motor` class implements a velocity control loop that runs at 200Hz (every 5ms).

### Control Equation
The final PWM output is a sum of Feedforward and Feedback terms:
$$PWM = (K_{ff} \times V_{target}) + PID(V_{error})$$

* **Feedforward ($K_{ff}$):** Calculates the theoretical PWM needed for the target velocity using `feedforward_gain_v`. This provides the "bulk" of the power.
* **Feedback ($PID$):** The `velocity_controller` compensates for battery voltage drop, friction, and load variance.
* **Deadband Handling:** The controller includes logic to zero out the PWM if the target velocity is effectively zero, preventing motor hum.

## 4. Maze Solver Logic (`solver.cpp`)
The robot uses a **Floodfill Algorithm** to navigate.

### A. Coordinate System & Wall Detection
* **Orientation Tracking:** The solver infers the robot's cardinal direction (North/South/East/West) based on its velocity variables (`bot_x_velocity`, `bot_y_velocity`).
* **Wall Updates:** Walls are detected relative to the robot (Front/Left/Right) using IR sensors and then mapped to absolute grid coordinates (North/South/East/West walls of specific cells).

### B. Decision Logic (`decideBestMove`)
The robot evaluates neighbors in the order: **Left $\rightarrow$ Right $\rightarrow$ Front**.
1.  It checks if a move is valid (no wall, within bounds).
2.  It compares the "Floodfill Distance" (steps to goal) of valid neighbors.
3.  It prioritizes the neighbor with the lowest distance value.

### C. State Machine
* **Init Phase:** The grid is initialized with the goal at the center (4 cells).
* **Search Phase:** The robot explores until it reaches the center `(Distance == 0)`.
* **Return Phase:** Once the goal is found, the goal coordinates are swapped to the Start Cell (0,0), and the robot floodfills its way back home.

## 5. Key Tuning Constants
| Parameter | File | Current Value | Description |
| :--- | :--- | :--- | :--- |
| **Gyro Scale** | `withGyroscope.ino` | `0.9935f` | Multiplier to fix sensor scale error. |
| **Speed Floor** | `withGyroscope.ino` | `3.0f` | Minimum turning speed (rad/s) when error > 15°. |
| **Turn PID** | `withGyroscope.ino` | `Kp=7e-3`, `Kd=1e-2` | PID gains for the final approach (<15° error). |
| **Drift PID** | `withGyroscope.ino` | `Kp=2.5`, `Kd=0.15` | Heading correction strength while driving straight. |
| **Straight Dist** | `withGyroscope.ino` | Time-based | Distance is currently estimated by `speed * time`. |# Micromouse Developer Reference (v2.0)

## 1. System Architecture Overview
This robot uses a hierarchical control system split into three layers:
1.  **High-Level Solver (`solver.cpp`):** Maintains the maze state and determines the next cardinal direction (North, South, East, West).
2.  **Mid-Level Navigator (`withGyroscope.ino`):** Converts cardinal moves into physical maneuvers (Turn 90°, Straight 1 cell) and manages the robot's heading.
3.  **Low-Level Actuator (`motor.cpp`):** Controls wheel velocity using feedforward and feedback loops.

## 2. Control System Implementation
Unlike standard textbook PID controllers, this robot uses a **Hybrid Control Strategy** to overcome mechanical stiction and sensor limitations.

### A. The Turning Controller (Hybrid Min-Speed + PID)
The robot does not rely solely on PID for turning. Instead, it uses a **Trapezoidal-like profile** enforced by logic clamps.

* **The Problem:** Pure PID control with small errors (e.g., 90° turn) generates low voltage, often failing to overcome gearbox static friction (stiction).
* **The Solution:** A "Speed Floor" forces the motors to move at a minimum speed until the robot is very close to the target.
* **Logic Flow:**
    1.  **Cruise Phase ($Error > 15^{\circ}$):** The controller overrides the PID output and forces `min_turn_speed` (set to 3.0 rad/s). This ensures consistent rotation regardless of the target angle.
    2.  **Landing Phase ($Error < 15^{\circ}$):** The logic hands control back to the PID controller to gently guide the robot to the stop point.
    3.  **Stop Threshold:** Control cuts off completely when error $< 0.5^{\circ}$ to prevent oscillation.

### B. Gyroscope Integration & Drift Compensation
The robot tracks its absolute heading (`position`) by integrating the Z-axis gyroscope data.

* **Drift Correction (Scalar):** A scalar value of `0.9935` is applied to the integration step to correct for sensor scale factor error (where the sensor reads faster/slower than reality). 
* **Drift Correction (Heading Hold):** When driving straight, a secondary PID loop (`drift_controller`) compares the current `position` against a locked `baseline_position`. It injects a differential speed adjustment (`correction`) to the motors to force the robot back to the centerline.

### C. Sensor Configuration
* **Range:** The BMI270 gyroscope is configured for **1000 dps** (Degrees Per Second).
    * *Note:* Turning speeds can exceed this range. Care should be taken to ensure the robot does not saturate the sensor during high-speed spins.
* **Filtering:** The Derivative (D) term of the PID controller utilizes a Low-Pass Filter (Bilinear Transform) to reject noise.
    * *Current Setting:* The cutoff frequency (`fc`) is currently set to **1.0 Hz** in `withGyroscope.ino`.

## 3. Motor Driver Implementation
The `Motor` class implements a velocity control loop that runs at 200Hz (every 5ms).

### Control Equation
The final PWM output is a sum of Feedforward and Feedback terms:
$$PWM = (K_{ff} \times V_{target}) + PID(V_{error})$$

* **Feedforward ($K_{ff}$):** Calculates the theoretical PWM needed for the target velocity using `feedforward_gain_v`. This provides the "bulk" of the power.
* **Feedback ($PID$):** The `velocity_controller` compensates for battery voltage drop, friction, and load variance.
* **Deadband Handling:** The controller includes logic to zero out the PWM if the target velocity is effectively zero, preventing motor hum.

## 4. Maze Solver Logic (`solver.cpp`)
The robot uses a **Floodfill Algorithm** to navigate.

### A. Coordinate System & Wall Detection
* **Orientation Tracking:** The solver infers the robot's cardinal direction (North/South/East/West) based on its velocity variables (`bot_x_velocity`, `bot_y_velocity`).
* **Wall Updates:** Walls are detected relative to the robot (Front/Left/Right) using IR sensors and then mapped to absolute grid coordinates (North/South/East/West walls of specific cells).

### B. Decision Logic (`decideBestMove`)
The robot evaluates neighbors in the order: **Left $\rightarrow$ Right $\rightarrow$ Front**.
1.  It checks if a move is valid (no wall, within bounds).
2.  It compares the "Floodfill Distance" (steps to goal) of valid neighbors.
3.  It prioritizes the neighbor with the lowest distance value.

### C. State Machine
* **Init Phase:** The grid is initialized with the goal at the center (4 cells).
* **Search Phase:** The robot explores until it reaches the center `(Distance == 0)`.
* **Return Phase:** Once the goal is found, the goal coordinates are swapped to the Start Cell (0,0), and the robot floodfills its way back home.

## 5. Key Tuning Constants
| Parameter | File | Current Value | Description |
| :--- | :--- | :--- | :--- |
| **Gyro Scale** | `withGyroscope.ino` | `0.9935f` | Multiplier to fix sensor scale error. |
| **Speed Floor** | `withGyroscope.ino` | `3.0f` | Minimum turning speed (rad/s) when error > 15°. |
| **Turn PID** | `withGyroscope.ino` | `Kp=7e-3`, `Kd=1e-2` | PID gains for the final approach (<15° error). |
| **Drift PID** | `withGyroscope.ino` | `Kp=2.5`, `Kd=0.15` | Heading correction strength while driving straight. |
| **Straight Dist** | `withGyroscope.ino` | Time-based | Distance is currently estimated by `speed * time`. |
