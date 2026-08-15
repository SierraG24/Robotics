# 3-Link Planar Robotic Arm

A Python-based forward and inverse kinematics project for a 3-link planar robotic arm.

The project started with a fixed end-effector orientation (`phi`) and was extended to experiment with **position-only inverse kinematics**, where `phi` is not known ahead of time.

The goal of the project is to understand how analytical inverse kinematics can be developed incrementally before moving toward more advanced robotic-arm control and eventually a rotating-base 3D arm.

---

## Features

* Forward kinematics
* Analytical inverse kinematics
* Relative joint-angle representation
* Elbow-up and elbow-down solutions
* Joint-limit checking
* Current joint configuration tracking
* Closest IK solution selection
* Fixed-`phi` inverse kinematics
* Variable-`phi` inverse kinematics
* Search through possible `phi` values
* Selection of the `phi` that produces the smallest joint movement
* Interactive Matplotlib visualization
* Mouse-based target selection

---

# Robot Configuration

The arm contains three links:

```text
                 Link 3
                   /
                  /
             Joint 3
                /
               /
        Link 2
          /
         /
    Joint 2
       /
      /
 Link 1
   /
  /
Base
```

The joint angles are **relative angles**:

* `theta1` = angle of link 1 relative to the +x axis
* `theta2` = angle of link 2 relative to link 1
* `theta3` = angle of link 3 relative to link 2

Therefore, the absolute orientation of each link is:

[
\theta_{1,abs}=\theta_1
]

[
\theta_{2,abs}=\theta_1+\theta_2
]

[
\theta_{3,abs}=\theta_1+\theta_2+\theta_3
]

The end-effector orientation is:

[
\boxed{
\phi=\theta_1+\theta_2+\theta_3
}
]

---

# Project Structure

### `config.py`

Contains the arm dimensions, joint limits, and fixed `phi` value.

### `kinematics.py`

Contains the mathematical model and forward/inverse kinematics algorithms.

### `visualization.py`

Contains the interactive Matplotlib visualization.

### `main.py`

Creates the kinematics model, initializes the arm, selects the IK mode, and starts the visualizer.

---

# Requirements

The project requires:

* Python 3
* NumPy
* Matplotlib

Install the dependencies with:

```bash
pip install numpy matplotlib
```

---

# Running the Project

From the project directory:

```bash
python main.py
```

A Matplotlib window should open showing the robotic arm.

Click inside the workspace to select an `(x, y)` target.

The inverse kinematics solver will attempt to move the arm to the selected location.

---

# Forward Kinematics

Forward kinematics starts with the joint angles and calculates the end-effector position.

The equations are:

[
x =
L_1\cos(\theta_1)
+
L_2\cos(\theta_1+\theta_2)
+
L_3\cos(\theta_1+\theta_2+\theta_3)
]

[
y =
L_1\sin(\theta_1)
+
L_2\sin(\theta_1+\theta_2)
+
L_3\sin(\theta_1+\theta_2+\theta_3)
]

The implementation is provided by:

```python
forwardKinematics(theta1, theta2, theta3)
```

---

# Inverse Kinematics

The 3-link arm has three unknown joint angles:

[
\theta_1,\theta_2,\theta_3
]

However, if only the end-effector position is specified:

[
(x,y)
]

there are only two constraints.

Therefore, the system has one degree of redundancy.

We use the end-effector orientation:

[
\phi
]

to represent this additional degree of freedom.

---

# Fixed-Phi IK

The first version of the 3-link solver assumes that `phi` is known.

For example:

```text
phi = 45°
```

The wrist position is calculated by removing link 3 from the target:

[
x_w=x-L_3\cos(\phi)
]

[
y_w=y-L_3\sin(\phi)
]

The first two links then solve a standard 2-link inverse kinematics problem.

After finding `theta1` and `theta2`, the third joint is calculated from:

[
\boxed{
\theta_3=\phi-\theta_1-\theta_2
}
]

This is implemented by:

```python
phiLockedIK(x, y)
```

---

# Elbow-Up and Elbow-Down

The first two links can generally reach the wrist in two configurations.

The two solutions are calculated using:

[
\theta_{1,down}=\alpha-\sigma
]

[
\theta_{2,down}=\pi-\beta
]

and:

[
\theta_{1,up}=\alpha+\sigma
]

[
\theta_{2,up}=\beta-\pi
]

Both configurations are checked against the joint limits.

---

# Variable-Phi IK

The next stage removes the requirement that `phi` be known beforehand.

There is no unique function:

[
\phi=f(x,y)
]

because a 3-link arm has three degrees of freedom while `(x,y)` provides only two constraints.

Instead, the program searches for possible values of `phi`.

---

## Phase 3 — Search for Phi

The function:

```python
findAllPhiSolutions()
```

searches through a range of possible orientations.

For example:

```python
phi_min = 0
phi_max = 360
phi_step = 1
```

The solver evaluates:

```text
0°
1°
2°
3°
...
360°
```

For each candidate `phi`:

1. Calculate the wrist position.
2. Check whether the wrist is reachable.
3. Solve the 2-link IK problem.
4. Calculate `theta3`.
5. Check all joint limits.
6. Store valid configurations.

The result is a collection of:

```text
(theta1, theta2, theta3, phi)
```

solutions.

---

# Phase 4 — Select the Best Phi

Finding every possible `phi` still leaves a question:

> Which configuration should the robot actually use?

The project chooses the configuration closest to the robot's current joint configuration.

The current configuration is:

[
q_{current}
===========

[
\theta_1,\theta_2,\theta_3
]
]

For every candidate solution, the joint movement is calculated.

The solution with the smallest overall joint movement is selected.

This is implemented by:

```python
findBestPhiValueIK(x, y)
```

Conceptually:

```text
Target
  ↓
Search phi
  ↓
Find valid configurations
  ↓
Compare with current configuration
  ↓
Choose smallest joint movement
  ↓
Move arm
```

Therefore, `phi` is not directly calculated from `(x,y)`.

Instead:

[
\boxed{
\phi^*=
\arg\min_{\phi\in\Phi_{valid}}
C(q)
}
]

where `C(q)` represents the distance from the current configuration.

---

# IK Modes

The visualizer supports two modes.

## Locked

```python
IK_MODE = "locked"
```

Uses the fixed `PHI` value from `config.py`.

The visualizer displays:

```text
IK: LOCKED
phi = 45°
```

---

## Search

```python
IK_MODE = "search"
```

Searches through possible `phi` values.

For example:

```python
phi_min=0.0
phi_max=360.0
phi_step=1.0
```

The visualizer displays the `phi` selected by the solver.

---

# Joint Limits

Every inverse-kinematics solution is checked against the defined joint limits.

This prevents mathematically valid configurations from being sent to the robot if they are outside the allowed mechanical range.

For example:

```python
THETA1_MIN = 0
THETA_MIN = -180
THETA_MAX = 180
```

The exact values can be changed in `config.py`.

---

# Current Configuration

The kinematics class stores the current joint configuration:

```python
setCurrentAngles(
    theta1,
    theta2,
    theta3
)
```

This is important for the variable-`phi` solver because the selected solution is based on the current configuration.

---

# Visualization

The visualizer provides:

* Robot links
* Joint locations
* Coordinate axes
* Workspace
* Target marker
* Interactive mouse target selection
* Current IK mode
* Selected `phi`

Clicking a point in the workspace runs the selected IK algorithm.

---

# Current Development Progress

The project has progressed through the following stages:

```text
Phase 1
Forward Kinematics
        ↓
Phase 2
Fixed-Phi Analytical IK
        ↓
Elbow-Up / Elbow-Down
        ↓
Joint Limits
        ↓
Closest Current Configuration
        ↓
Phase 3
Search for Phi
        ↓
Phase 4
Choose Best Phi
```
---
