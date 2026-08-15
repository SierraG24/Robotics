# 2-Link Planar Robotic Arm

A Python-based forward and inverse kinematics project for a 2-link planar robotic arm.

This project was developed as the foundation for the later 3-link arm. It focuses on understanding analytical inverse kinematics, geometric solutions, elbow-up/elbow-down configurations, joint limits, and selecting the solution closest to the current robot configuration.

---

## Features

* Forward kinematics
* Analytical inverse kinematics
* Geometric IK solution
* Elbow-up configuration
* Elbow-down configuration
* Joint-limit checking
* Current joint configuration tracking
* Closest-solution selection
* Interactive Matplotlib visualization
* Mouse-based target selection
* Workspace visualization

---

# Robot Configuration

The robot contains two links:

```text
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

The joint angles are defined as relative angles:

* `theta1` = angle of link 1 from the +x axis
* `theta2` = angle of link 2 relative to link 1

Therefore, the absolute orientation of link 2 is:

[
\boxed{
\theta_{2,abs}=\theta_1+\theta_2
}
]


---

# Project Structure

### `config.py`

Contains:

* Link lengths
* Joint limits

### `kinematics.py`

Contains:

* Forward kinematics
* Inverse kinematics
* Joint-limit checking
* Current joint configuration

### `visualization.py`

Contains the interactive Matplotlib visualization.

### `main.py`

Creates the robot and starts the visualizer.

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

A Matplotlib window will open.

Click inside the workspace to select an `(x,y)` target.

The inverse kinematics solver will calculate a joint configuration that reaches the target.

---

# Forward Kinematics

Forward kinematics calculates the end-effector position from the joint angles.

For the first link:

[
x_1=L_1\cos(\theta_1)
]

[
y_1=L_1\sin(\theta_1)
]

The end effector is:

[
x=
L_1\cos(\theta_1)
+
L_2\cos(\theta_1+\theta_2)
]

[
y=
L_1\sin(\theta_1)
+
L_2\sin(\theta_1+\theta_2)
]

---

# Inverse Kinematics

For a desired target:

[
(x,y)
]

the solver first calculates the distance from the base to the target:

[
r=\sqrt{x^2+y^2}
]

The target is reachable only when:

[
|L_1-L_2|
\leq r
\leq L_1+L_2
]

If the target is outside this region, there is no physical solution.

---

# Geometric IK

The inverse kinematics solution uses the law of cosines.

The angle between the two links is determined using:

[
\cos(\beta)
===========

\frac{
L_1^2+L_2^2-r^2
}{
2L_1L_2
}
]

The angle between the first link and the line from the base to the target is:

[
\cos(\sigma)
============

\frac{
r^2+L_1^2-L_2^2
}{
2L_1r
}
]

The target direction is:

[
\alpha=
\operatorname{atan2}(y,x)
]

These angles are combined to produce the two possible configurations.

---

# Elbow-Down Solution

One configuration is:

[
\boxed{
\theta_1=\alpha-\sigma
}
]

[
\boxed{
\theta_2=\pi-\beta
}
]

This produces one orientation of the elbow.

---

# Elbow-Up Solution

The second configuration is:

[
\boxed{
\theta_1=\alpha+\sigma
}
]

[
\boxed{
\theta_2=\beta-\pi
}
]

Both solutions reach the same end-effector position.

---

# Why Are There Two Solutions?

For most reachable points, the elbow can be placed on either side of the line between the base and the target.

Conceptually:

```text
Elbow-Up

      Joint 2
       /
      /
Base /____________ Target
```

and:

```text
Base \____________ Target
       \
        \
       Joint 2

Elbow-Down
```

Both configurations produce the same end-effector position.

---

# Joint Limits

Not every mathematical solution is physically usable.

The solver checks:

```python
theta1_min <= theta1 <= theta1_max
```

and:

```python
theta2_min <= theta2 <= theta2_max
```

Solutions outside these limits are rejected.

This is particularly important when the arm has a restricted first joint, such as:

[
0^\circ\leq\theta_1\leq180^\circ
]

---

# Closest Solution

If both elbow-up and elbow-down solutions are valid, the solver compares them against the robot's current configuration.

For example, if the robot is currently:

[
q_{current}
===========

[\theta_1,\theta_2]
]

the solver calculates the difference between the current and candidate configurations.

The configuration requiring the least joint movement is selected.

Conceptually:

```text
Target
  ↓
Find elbow-up solution
Find elbow-down solution
  ↓
Check joint limits
  ↓
Compare with current configuration
  ↓
Choose closest valid solution
```

This prevents the arm from unnecessarily switching between elbow configurations.

---

# Current Joint Configuration

The kinematics class stores the current angles:

```python
setCurrentAngles(
    theta1,
    theta2
)
```

The current values can be retrieved with:

```python
getCurrentAngles()
```

This state is used when selecting the closest IK solution.

---

# Visualization

The visualizer provides:

* Two robot links
* Joint locations
* Coordinate axes
* Workspace boundaries
* Target marker
* Interactive mouse target selection
* Elbow-up/elbow-down IK
* Joint-limit visualization
* Current configuration tracking

Clicking inside the workspace sends the selected target to the IK solver.

---

# Development Progress

The project was developed in stages:

```text
Phase 1
Forward Kinematics
        ↓
Phase 2
Geometric Inverse Kinematics
        ↓
Elbow-Up / Elbow-Down
        ↓
Joint Limits
        ↓
Closest-to-Current Solution
        ↓
Interactive Visualization
        ↓
Foundation for 3-Link Arm
```

---

# Connection to the 3-Link Arm

The 2-link arm became the mathematical foundation for the 3-link arm.

The key idea was to temporarily remove the third link by calculating the wrist position.

For the 3-link arm:

[
x_w=x-L_3\cos(\phi)
]

[
y_w=y-L_3\sin(\phi)
]

The first two links can then use the same 2-link IK equations developed in this project.

Therefore:

```text
2-Link IK
    │
    ├── alpha
    ├── beta
    ├── sigma
    ├── elbow-up
    └── elbow-down
             │
             ▼
       3-Link IK
             │
             ├── Find wrist
             ├── Use 2-Link IK
             └── Solve theta3
```

The 2-link project is therefore the mathematical foundation for the larger robotic-arm project.

---
