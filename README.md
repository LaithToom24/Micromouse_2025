# Micromouse Robot — Firmware Documentation

A complete firmware implementation for an autonomous micromouse robot capable of navigating and solving an 8×8 maze using a flood-fill algorithm. The robot uses Time-of-Flight distance sensors for wall detection, a BMI270 IMU for gyroscope-based heading control, and dual DC motors with quadrature encoders controlled by cascaded PID loops.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Hardware](#hardware)
- [Software Architecture](#software-architecture)
- [Module Reference](#module-reference)
  - [Main Entry Point — `WINNING_CODE.ino`](#main-entry-point--winning_codeino)
  - [Maze Solver — `solver.cpp / solver.hpp`](#maze-solver--solvercpp--solverhpp)
  - [Motor Control — `motor.cpp / motor.hpp`](#motor-control--motorcpp--motorhpp)
  - [PID Controller — `PID_controller.cpp / PID_controller.hpp`](#pid-controller--pid_controllercpp--pid_controllerhpp)
  - [Sensor — `sensor.cpp / sensor.hpp`](#sensor--sensorcpp--sensorhpp)
  - [Gyroscope — `gyroscope.cpp / gyroscope.hpp`](#gyroscope--gyroscopecpp--gyroscopehpp)
  - [Command Queue — `commands.cpp / commands.hpp`](#command-queue--commandscpp--commandshpp)
  - [API — `API.cpp / API.hpp`](#api--apicpp--apihpp)
- [Control Flow](#control-flow)
- [PID Tuning Parameters](#pid-tuning-parameters)
- [Pin Assignments](#pin-assignments)

---

## Project Overview

This firmware implements a fully autonomous micromouse: a small wheeled robot that explores and solves a 16×16-cell-compatible maze (configured here for 8×8). The robot begins at the origin cell `(0, 0)` and navigates to the four central goal cells. Once the goal is reached, it resets its flood-fill target back to the origin and runs the return path, toggling between the two targets indefinitely to achieve an optimal route.

**Core capabilities:**

- Real-time flood-fill pathfinding with dynamic wall discovery
- Dual closed-loop PID velocity control (one controller per motor)
- Gyroscope-integrated heading hold and turn execution
- Time-of-Flight (ToF) wall detection with confidence filtering
- Non-blocking command queue and sensor settle logic
- Hardware diagnostic modes for sensors and gyroscope

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Microcontroller | Arduino-compatible (AVR/ARM) | Uses `micros()`, `analogWrite()`, and `Wire` |
| IMU | SparkFun BMI270 | I²C address `0x68`; gyro + accel |
| Distance Sensors | Adafruit VL53L0X × 3 | Left, Front, Right; I²C with XSHUT GPIO control |
| Motors | DC Gear Motors × 2 | 12 CPR encoder, 15:1 gear ratio, 3 cm wheel diameter |
| Motor Driver | TB6612 or equivalent | STBY pin on `D8`; dual H-bridge |
| Indicator LEDs | 3 × LEDs | On `A0` (left), `A6` (front), `A7` (right) |
| Mode Switch | Tactile switch | On `D4`; enables hardware diagnostic on boot |

---

## Software Architecture

```
WINNING_CODE.ino          ← Arduino entry point (setup + loop)
│
├── solver_loop()         ← Calls flood-fill solver, enqueues next move
├── command_loop()        ← Executes straight/turn commands from queue
├── sensor_loop()         ← Polls ToF sensors with confidence filtering
└── update_position()     ← Integrates gyro Z for absolute heading
    │
    ├── solver.cpp        ← Flood-fill maze solver + wall detection
    ├── API.cpp           ← Translates solver actions into queue commands
    ├── commands.cpp      ← FIFO command queue
    ├── motor.cpp         ← Per-motor velocity PID + encoder odometry
    ├── PID_controller.cpp← Reusable discrete PID with filtered derivative
    ├── sensor.cpp        ← VL53L0X driver (3-sensor I²C setup)
    └── gyroscope.cpp     ← BMI270 driver + bias calibration
```

The main loop runs fully non-blocking. Sensor polling, motor control, and solver decisions each operate on independent timers checked with `micros()`, so no single subsystem stalls the others.

---

## Module Reference

### Main Entry Point — `WINNING_CODE.ino`

This is the Arduino sketch that wires every module together.

**`setup()`**

Initialises all peripherals in this order:

1. Serial at 115200 baud
2. LED indicators (A0, A6, A7) with a startup light sequence
3. Motor driver STBY pin (`D8`)
4. Left and right motor objects (pins, specs, PID gains, orientation)
5. BMI270 gyroscope (`gyro_init()`) — blocks until calibration completes; the robot must be stationary
6. VL53L0X sensors (`ToF_setup()`) — assigns I²C addresses sequentially and starts continuous ranging
7. If the boot switch on `D4` is HIGH, runs hardware diagnostics (`ToF_test()` then `gyro_test()`)

**`loop()`**

Runs four tasks every iteration:

- `update_position(now)` — gyro-integrates heading (runs every 100 µs)
- `sensor_loop()` — reads ToF sensors with confidence gating (runs every 5 ms)
- `solver_loop()` — calls flood-fill and pushes commands (runs every 10 ms, only when the queue is idle)
- `command_loop()` — executes the front command in the queue (runs every iteration)

**`straight(float distance)`** → `bool`

Drives the robot forward a target distance (in cm) using wheel-encoder odometry. While travelling:

- A gyro-based **drift correction** PID adjusts differential motor speeds to maintain heading.
- A **wall proximity correction** term nudges the robot away from any side wall closer than 35 mm.
- Speed is reduced proportionally when a front wall is within 250 mm.
- An **emergency abort** triggers if the front sensor reads less than 45 mm, preventing collisions when encoder distance hasn't been reached yet.

Returns `true` when the target distance is covered or the abort condition fires.

**`turn(float degrees)`** → `bool`

Rotates the robot in place by the given number of degrees (positive = right, negative = left) using the gyroscope-integrated heading and a PID turning controller. A **friction feedforward** term of ±0.53 m/s and a minimum velocity floor of 0.75 m/s prevent the motors from stalling near the target. Returns `true` when the heading error is within ±2°.

**`sensor_loop()`**

Polls all three VL53L0X sensors every 5 ms and applies **confidence filtering**: a wall is only declared present after two consecutive readings below the threshold (100 mm front, 85 mm left/right). This suppresses single-sample glitches. LED indicators on A0/A6/A7 mirror the current wall state.

**`update_position()`**

Integrates gyroscope Z angular velocity (with a small empirical correction factor of 0.9935) every 100 µs to maintain the robot's absolute heading in degrees. This is the ground truth for both `turn()` and the drift correction in `straight()`.

**`command_loop()`**

Executes the head of the command queue. After a command finishes, it enters a **non-blocking settle phase** of 33 ms (controlled by `is_waiting` and `wait_start_time`) before marking `performing_command = false` and allowing the solver to read fresh sensor data. This prevents the solver from acting on stale readings taken mid-movement.

---

### Maze Solver — `solver.cpp / solver.hpp`

Implements an 8×8 flood-fill solver. The maze is represented as a flat array of `Cell` structs indexed by `x + LENGTH * y`.

**Data structures**

```cpp
typedef struct Cell {
    uint8_t  Coordinate[2]; // [x, y]
    uint16_t Distance;       // flood-fill cost to goal
    bool     walls[4];       // [North, East, South, West]
    bool     Filled;         // true if already assigned a distance
};
```

The solver maintains a BFS queue (`queue[]`, max 512 entries) used during flood-fill propagation.

**Key global state**

| Variable | Description |
|---|---|
| `bot_x_pos`, `bot_y_pos` | Robot's current grid position |
| `bot_x_velocity`, `bot_y_velocity` | Robot's heading as a unit direction vector |
| `goal_cells_existing` | 4 = navigate to centre; 1 = navigate back to origin |
| `goal_reached` | True once the robot has entered a goal cell |

**`init_grid()`**

Called once on the first solver invocation. Clears all cells, sets the outer perimeter walls (which always exist in a standard maze), seeds the four central goal cells with distance 0, and runs flood-fill to completion.

**`solver(bool front, bool left, bool right)`** → `Action`

Top-level entry point called once per solver tick. Calls `floodFill()` after initialising the grid on the first call.

**`floodFill(bool front, bool left, bool right)`** → `Action`

The main decision loop:

1. Calls `detectWalls()` with the three sensor booleans.
2. If new walls were found, calls `recalculateFloodfill()`.
3. Calls `decideBestMove()` to select the lowest-distance neighbour.
4. Tracks goal arrival and toggles the target between the centre and origin.
5. Updates `bot_x_pos`, `bot_y_pos`, and the direction velocity vector to reflect the chosen move.

Returns one of `FORWARD`, `LEFT`, `RIGHT`, or `IDLE`.

**`detectWalls(bool front, bool left, bool right)`** → `bool`

Translates the robot-relative sensor readings (front/left/right) into absolute compass directions based on the current heading vector, then calls `setWall()` for each detected wall. Returns `true` if any wall was newly discovered.

**`setWall(int x, int y, int dir)`** → `bool`

Sets a wall on cell `(x, y)` in direction `dir` and **mirrors** it to the opposite face of the adjacent cell (e.g. setting the North wall of cell A also sets the South wall of cell A's northern neighbour). Returns `true` only if the wall was not previously recorded, allowing the caller to know whether a recalculation is needed.

**`recalculateFloodfill()`**

Resets all cell distances and `Filled` flags (preserving wall data), re-seeds the goal cells, and re-runs the BFS to completion.

**`serviceQueue()`**

Processes one entry from the BFS queue: assigns `Distance = parent.Distance + 1` to each reachable, un-filled neighbour and pushes them onto the queue. Wall checks are bidirectional (both cells must agree there is no wall between them).

**`decideBestMove()`** → `Action`

Reads the flood-fill distances of the three accessible neighbours (left, right, front) relative to the robot's current heading. Returns the direction pointing to the cell with the lowest distance. Inaccessible cells (wall present or out of bounds) are given a distance of 1000 so they are never chosen.

---

### Motor Control — `motor.cpp / motor.hpp`

The `Motor` class encapsulates a single DC gear motor with quadrature encoder feedback and closed-loop velocity control.

**Construction and setup**

```cpp
Motor m;
m.set_motor_pins(pwm_pin, in1, in2, encoder_a, encoder_b);
m.set_interrupt(isr_function);
m.set_motor_specs(cpr, gear_ratio, wheel_diameter_cm, max_vel);
m.set_velocity_controls(kf, kp, ki, kd, ctrl_period_us, meas_rate, cutoff_hz);
m.set_motor_orientation(true);  // true = left motor
```

**`set_vel(float vel)`**

Sets the target velocity (cm/s). Implements a **feedforward + feedback** architecture:

- Feedforward: converts target velocity to a baseline PWM using `feedforward_gain_v`.
- Feedback: a `PID_Controller` instance corrects the error between target and measured velocity. The integrator is cleared and PID is skipped when `|vel| < 0.1` (effectively a stop command).

The directional logic for `IN1`/`IN2` is automatically inverted for the right motor based on the `LEFT` flag.

**`readEncoder()`** (ISR)

Called from a pin-change interrupt on `ENCODER_A`. Implements 2× quadrature decoding: compares the state of channel A and B to determine rotation direction, then increments or decrements both `encoder_count` (used for velocity measurement, reset each measurement window) and `rotational_encoder_count` (absolute position, never reset unless explicitly called).

**`get_distance()`** → `float`

Returns total distance travelled (cm) since the last `reset_position()` call, computed from `rotational_encoder_count`.

**`update_vel()`** → `float`

Called internally at the configured measurement rate. Samples `encoder_count` atomically (interrupts disabled), resets it to zero, and converts the count to a velocity using the pre-computed `count_to_vel` factor.

---

### PID Controller — `PID_controller.cpp / PID_controller.hpp`

A general-purpose discrete-time PID controller with a **filtered derivative** and **anti-windup** integrator.

**`setup(float Kp, float Ki, float Kd, int ctrl_period_us, int var_samp_time, float cutoff_hz, float max_control)`**

Configures the controller. `ctrl_period_us` is the control loop period in microseconds. `var_samp_time` controls how many control loops pass between derivative measurements (reduces noise). `cutoff_hz` sets the derivative low-pass filter cutoff.

**`process(float error, float measured, unsigned long now)`** → `float`

Should be called every loop iteration. Internally rate-limits itself to `ctrl_period_us`. Computes:

- **Proportional:** `kp * error`
- **Derivative:** bilinear-transform filtered differentiator applied to the measured variable (avoids derivative kick on setpoint changes): `deriv[n] = b_d * deriv[n-1] + a_d * (measured[n] - measured[n-1])`
- **Integral:** trapezoidal integration: `integral[n] = integral[n-1] + a_i * (error[n] + error[n-1])`
- **Anti-windup:** the integral is clamped to `[-max_control/Ki, +max_control/Ki]`

The final output is clamped to `[-max_control, +max_control]`.

**`clear()`**

Resets the integral, derivative, and error state. Call this between distinct motion commands to prevent accumulated state from the previous command affecting the next.

---

### Sensor — `sensor.cpp / sensor.hpp`

Manages three VL53L0X Time-of-Flight ranging sensors (left, front, right) over I²C.

**I²C address assignment**

All three sensors share the same default I²C address at power-on. `setID()` reassigns them sequentially by toggling their individual XSHUT pins (`A1`, `A2`, `A3`):

| Sensor | Object | I²C Address | Physical Position |
|---|---|---|---|
| lox1 | `Adafruit_VL53L0X` | `0x30` | Right |
| lox2 | `Adafruit_VL53L0X` | `0x31` | Front |
| lox3 | `Adafruit_VL53L0X` | `0x32` | Left |

**`ToF_setup()`**

Configures XSHUT pins, calls `setID()`, and starts continuous ranging on all three sensors.

**`readLeft()` / `readRight()` / `readFront()`** → `int`

Non-blocking reads using `isRangeComplete()`. Returns the last valid reading in millimetres. Readings above 8000 (the VL53L0X out-of-range error code) are clamped to 1000 mm. Each function retains the last good reading as a static variable so stale-but-valid data is returned when no new measurement is ready.

**`ToF_test()`**

Diagnostic routine. Sequentially waits for a wall to be brought within 100 mm of each sensor and lights the corresponding LED indicator when confirmed. Ends with a 2× blink of all three LEDs.

---

### Gyroscope — `gyroscope.cpp / gyroscope.hpp`

Wraps the SparkFun BMI270 library to provide angular velocity readings.

**`gyro_init()`**

Initialises the BMI270 over I²C at 400 kHz. Configures:

- Accelerometer: 800 Hz ODR, OSR4 filter, 2 g range
- Gyroscope: 800 Hz ODR, OSR4 filter, 1000 °/s range, performance mode

After configuration, **calculates a static bias** by averaging 500 gyro-Z samples over ~1 second while the robot is stationary. This bias is subtracted from every subsequent reading.

> **Important:** The robot must not be moved during `gyro_init()`. The maze-run start is delayed until bias calibration completes.

**`get_gyroZ()`** → `float`

Returns the bias-corrected Z-axis angular velocity in degrees per second.

**`gyro_test()`**

Diagnostic routine. Re-runs `gyro_init()`, then integrates heading in a loop and blinks an LED every 90° of manual rotation. Exits after one 90° threshold is crossed.

---

### Command Queue — `commands.cpp / commands.hpp`

A simple fixed-size FIFO queue (capacity: 20 commands) that decouples the solver from the motor execution layer.

**`Command` struct**

```cpp
typedef struct Command {
    bool  type;   // 0 = straight,  1 = turn
    float value;  // cm for straight, degrees for turn
};
```

**`add_command(bool type, float value)`**

Appends a command to the back of the queue. Silently drops commands if the queue is full.

**`remove_command()`**

Removes the front command by shifting all elements down. Called by `command_loop()` after a command completes.

---

### API — `API.cpp / API.hpp`

A thin translation layer between the solver's abstract actions and the command queue. Isolates the solver from low-level motion parameters.

| Function | Command enqueued |
|---|---|
| `API_moveForward()` | Straight, 2.0 cm |
| `API_turnLeft()` | Turn, −90° |
| `API_turnRight()` | Turn, +90° |

`API_wallFront()`, `API_wallLeft()`, and `API_wallRight()` are declared in `API.hpp` and implemented in the main `.ino` file (they simply read the globally-filtered `wallFront`/`wallLeft`/`wallRight` booleans).

---

## Control Flow

```
loop()
 │
 ├─ update_position()          Every 100 µs
 │    └─ integrate get_gyroZ() → position (degrees)
 │
 ├─ sensor_loop()              Every 5 ms
 │    ├─ readLeft/Right/Front()
 │    └─ confidence filter → wallLeft, wallRight, wallFront
 │
 ├─ solver_loop()              Every 10 ms, only when queue is empty
 │    ├─ solver(wallFront, wallLeft, wallRight)
 │    │    ├─ detectWalls()  → setWall() + recalculateFloodfill()
 │    │    └─ decideBestMove() → FORWARD / LEFT / RIGHT
 │    └─ API_moveForward() / API_turnLeft() / API_turnRight()
 │         └─ add_command() → command_queue[]
 │
 └─ command_loop()             Every iteration
      ├─ straight(dist) or turn(degrees)
      │    ├─ straight(): encoder odometry + gyro drift correction
      │    └─ turn():     gyro-integrated heading + PID
      ├─ remove_command() on completion
      └─ 33 ms non-blocking settle delay before unlocking solver
```

---

## PID Tuning Parameters

All tuning constants are defined at the top of `WINNING_CODE.ino`.

**Velocity controller** (per motor)

| Parameter | Value | Description |
|---|---|---|
| `kf_v` | 1.0 | Feedforward gain (fraction of max PWM per max velocity) |
| `kp_v` | 0.1 | Proportional gain |
| `ki_v` | 0.01 | Integral gain |
| `kd_v` | 1×10⁻⁴ | Derivative gain |

**Turning controller** (gyro-based, shared)

| Parameter | Value | Description |
|---|---|---|
| `kp_p` | 4.5×10⁻³ | Proportional gain |
| `ki_p` | 2.75×10⁻³ | Integral gain |
| `kd_p` | 1.25×10⁻³ | Derivative gain |
| Max output | ±0.95 m/s | Clamped motor velocity |

**Drift controller** (straight-line heading hold)

| Parameter | Value |
|---|---|
| Kp | 5×10⁻³ |
| Ki | 5×10⁻³ |
| Kd | 1×10⁻⁷ |
| Cutoff | 9 Hz |
| Max output | ±0.325 m/s |

---

## Pin Assignments

| Pin | Function |
|---|---|
| `D2` | Right motor encoder channel A (interrupt) |
| `D3` | Right motor encoder channel B |
| `D4` | Boot mode switch (HIGH = run diagnostics) |
| `D5` | Right motor PWM |
| `D6` | Right motor IN1 |
| `D7` | Right motor IN2 |
| `D8` | Motor driver STBY (active HIGH) |
| `D9` | Left motor IN1 |
| `D10` | Left motor IN2 |
| `D11` | Left motor PWM |
| `D12` | Left motor encoder channel A (interrupt) |
| `D13` | Left motor encoder channel B |
| `A0` | LED indicator — Left wall |
| `A1` | VL53L0X XSHUT — Right sensor |
| `A2` | VL53L0X XSHUT — Front sensor |
| `A3` | VL53L0X XSHUT — Left sensor |
| `A6` | LED indicator — Front wall |
| `A7` | LED indicator — Right wall |
| `SDA/SCL` | I²C bus (BMI270 + 3× VL53L0X) |
