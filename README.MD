# PID Path Following Robot

A Python-based simulation of a mobile robot following reference paths using a PID controller.

This project was created to understand **feedback control, path tracking, controller tuning, and mobile robot behavior** before implementing a controller on physical robot hardware.

The project progressively builds a PID-based path-following system, including reference-path generation, nearest-point tracking, cross-track error calculation, PID control, velocity effects, controller tuning, and RMSE performance evaluation.

---

## Simulation

![PID Path Following Simulation](/images/Config_Pid/config_p/Sine/p_3.5.png)

The simulation allows the robot's motion and controller behavior to be observed while changing the reference path, robot velocity, and PID parameters.

---

# Features

* PID controller
* Proportional, integral, and derivative control
* Integral windup protection
* Multiple reference paths
* Nearest-point path tracking
* Cross-track error calculation
* Simulated mobile robot
* Interactive Matplotlib visualization
* Adjustable robot velocity
* Configurable PID parameters
* RMSE performance evaluation
* PID tuning experimentation
* Comparison of controller behavior at different velocities

---

# Requirements

The project requires:

* Python 3
* NumPy
* Matplotlib

Install the dependencies:

```bash
pip install numpy matplotlib
```

---

# Running the Project

From the project directory:

```bash
python main.py
```

The simulation will open a Matplotlib window showing the robot following the selected reference path.

PID parameters, robot velocity, and reference-path settings can be modified through the project's configuration files.

---

# How the Controller Works

The controller continuously compares the robot's current position to a desired reference path.

The overall process is:

```text
Reference Path
      |
      v
Find Nearest Point
      |
      v
Calculate Tracking Error
      |
      v
PID Controller
      |
      v
Calculate Steering
      |
      v
Update Robot
      |
      v
Repeat
```

At each simulation step, the robot finds the closest point on the reference path and calculates its tracking error.

The PID controller uses this error to determine how the robot should steer.

The process repeats continuously as the robot moves along the path.

---

# Discrete PID Controller

Because the simulation operates in discrete time, the continuous PID equation is approximated numerically.

The integral term accumulates the tracking error over time:

[
I(k)=I(k-1)+e(k)\Delta t
]

The derivative term measures how quickly the error is changing:

[
D(k)=\frac{e(k)-e(k-1)}{\Delta t}
]

The final control signal is:

[
u(k)=K_Pe(k)+K_I I(k)+K_DD(k)
]

Where:

* (e(k)) = tracking error
* (K_P) = proportional gain
* (K_I) = integral gain
* (K_D) = derivative gain
* (\Delta t) = simulation time step
* (u(k)) = steering control signal

---

# Reference Paths

The project supports multiple reference paths for testing the PID controller.

## Straight

A straight path is useful for initial controller testing.

```text
------------------------------------>
```

This provides a simple environment for checking whether the controller can converge to the path.

---

## Sine

A sinusoidal path introduces continuously changing curvature.

```text
       /\        /\
      /  \      /  \
-----/----\----/----\-----
```

This is useful for testing how well the controller responds to continuous changes in direction.

---

## Circle

A circular path requires the robot to continuously turn.

```text
        ****
      **    **
     *        *
      **    **
        ****
```

This tests whether the controller can maintain tracking during continuous motion.

---

## Square

A square path introduces sharp changes in direction.

```text
+-------------+
|             |
|             |
|             |
+-------------+
```

This is useful for testing the controller's response to sudden changes in path direction.

---

## Trapezoid

The trapezoid path provides another shape with changing curvature and sharp corners.

```text
       ______
      /      \
     /        \
    /__________\
```

It is useful for testing controller behavior under different geometric conditions.

---

# Robot Velocity Effects

Robot velocity is an important part of the simulation because changing the velocity changes how quickly the robot responds to the reference path.

At lower velocities, the controller generally has more time to correct tracking errors.

At higher velocities, the robot moves farther between control updates, making aggressive turns more difficult to track.

This can result in:

* Increased cross-track error
* Larger overshoot
* Increased oscillation
* Difficulty following sharp corners
* Greater sensitivity to PID parameters

Therefore, PID gains that work well at one velocity may not perform equally well at another.

The project uses velocity changes to investigate the relationship between **controller tuning and robot dynamics**.

---


# Performance Evaluation

The controller's performance is evaluated using **Root Mean Square Error (RMSE)**.

The RMSE is calculated as:

[
RMSE=\sqrt{\frac{1}{N}\sum_{k=1}^{N}e_k^2}
]

Where:

* (N) = number of simulation samples
* (e_k) = tracking error at sample (k)

A lower RMSE indicates that the robot stayed closer to the reference path on average.

RMSE can be used to compare:

* Different PID gains
* Different reference paths
* Different robot velocities
* Different tuning strategies

This provides a quantitative way to evaluate the controller rather than relying only on visual inspection.

---

# Experiments

The project can be used to investigate several control-system questions.

### PID Gain Experiment

Change (K_P), (K_I), and (K_D) and observe how the robot's behavior changes.

Questions to investigate:

* How does increasing (K_P) affect oscillation?
* How does (K_D) affect overshoot?
* How much integral control is necessary?
* What happens when the integral gain is too large?

### Velocity Experiment

Keep the PID gains constant while changing the robot velocity.

Compare:

```text
Low Velocity
     ↓
Smaller Tracking Error
     ↓
Higher Velocity
     ↓
Larger Tracking Error
```

This demonstrates why controller tuning cannot always be separated from the motion characteristics of the robot.

### Path Experiment

Use the same PID parameters on different paths.

For example:

```text
Straight → Sine → Circle → Trapezoid → Square
```

The resulting RMSE values can be compared to determine which path geometries are more difficult for the controller to follow.

---

# Project Progression

The project is designed to progressively introduce concepts used in mobile robotics and feedback control.

```text
Reference Path Generation
          |
          v
Nearest-Point Tracking
          |
          v
Tracking Error
          |
          v
PID Controller
          |
          v
Robot Motion
          |
          v
Velocity Effects
          |
          v
PID Tuning
          |
          v
RMSE Evaluation
          |
          v
Controller Analysis
```

This progression provides a foundation for eventually implementing similar control algorithms on physical robotic systems.

---

# Project Goal

The ultimate goal of this project is to bridge the gap between **control theory, simulation, and physical robotics**.

The simulation provides a controlled environment for understanding how PID feedback behaves before dealing with real-world effects such as sensor noise, actuator limitations, motor dynamics, communication delays, and mechanical constraints.
