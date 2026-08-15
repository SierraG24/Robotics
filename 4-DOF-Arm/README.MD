# 4-DOF Robotic Arm Kinematics

A Python-based simulation and visualization project for a 4-DOF robotic arm. The project focuses on understanding and implementing forward kinematics, inverse kinematics, joint limits, and interactive 3D visualization.

The arm consists of three planar links mounted on a revolute base joint, giving the system four degrees of freedom:

* Base rotation
* Shoulder joint
* Elbow joint
* Wrist joint

The project was developed incrementally to better understand how robotic arm geometry, forward kinematics, and inverse kinematics work before moving toward a physical robotic arm implementation.

---

## Project Goals

The main goals of this project are to:

* Implement forward kinematics for a 4-DOF robotic arm.
* Implement inverse kinematics for a desired 3D endpoint.
* Account for joint angle limits.
* Handle multiple possible IK configurations.
* Search for a suitable end-effector orientation.
* Select the IK configuration closest to the arm's current position.
* Build an interactive 3D visualization for testing the kinematics.
* Separate the mathematical kinematics code from the visualization code.

---

## Robot Configuration

The robotic arm uses three links:

```text
L1 = 1.75
L2 = 1.25
L3 = 1.00
```

The four joint variables are:

```text
theta_base
theta1
theta2
theta3
```

The base joint rotates the planar arm around the Z-axis.

The remaining joints are defined as relative angles:

```text
Link 1 absolute angle = theta1

Link 2 absolute angle = theta1 + theta2

Link 3 absolute angle = theta1 + theta2 + theta3
```

The end-effector orientation is therefore:

```text
phi = theta1 + theta2 + theta3
```

---

## Joint Limits

The current joint limits are:

```text
Base:
    -180° to 180°

Theta 1:
       0° to 180°

Theta 2:
    -180° to 180°

Theta 3:
    -180° to 180°
```

These limits are checked whenever a joint configuration is applied.

If a joint exceeds its allowed range, the kinematics class raises a `ValueError`.

---

## Forward Kinematics

Forward kinematics calculates the position of every joint and the end effector from the current joint angles.

The base rotation is first used to determine the direction of the arm in the XY plane.

The arm itself operates in a vertical plane defined by the base angle.

For each link, the radial distance and vertical position are calculated and then rotated around the Z-axis using `theta_base`.

The forward kinematics function returns:

```text
[
    Base,
    Joint 1,
    Joint 2,
    End Effector
]
```

with each position represented as:

```text
(x, y, z)
```

This allows the visualization to draw the complete arm rather than only the end-effector position.

---

## Inverse Kinematics

The inverse kinematics system starts with a desired endpoint:

```text
(x, y, z)
```

and determines a set of joint angles capable of reaching that position.

### 1. Base Rotation

The base angle is determined from:

```text
theta_base = atan2(y, x)
```

This rotates the planar arm toward the target in the XY plane.

### 2. Planar Projection

The target is projected into the arm's working plane using:

```text
r = sqrt(x² + y²)
```

The resulting planar target becomes:

```text
(r, z)
```

### 3. End-Effector Orientation

Because the endpoint does not uniquely determine the orientation of the end effector, the solver searches over possible values of:

```text
phi
```

The current implementation searches:

```text
0° to 360°
```

with a default step size of:

```text
1°
```

### 4. Wrist Position

For each candidate `phi`, the wrist position is calculated by removing the final link from the desired endpoint:

```text
r_wrist = r - L3 cos(phi)

z_wrist = z - L3 sin(phi)
```

This converts the 3-link problem into a 2-link problem.

### 5. Two-Link Inverse Kinematics

The wrist position is solved using the law of cosines.

Two possible configurations are calculated:

```text
Elbow Down
Elbow Up
```

For each configuration, `theta3` is calculated from:

```text
theta3 = phi - theta1 - theta2
```

### 6. Joint Limit Validation

Every candidate solution is checked against the configured joint limits.

Invalid configurations are discarded.

### 7. Best Solution Selection

The solver searches through every valid `phi` and both elbow configurations.

The final solution is selected based on which configuration is closest to the robot's current joint configuration.

The result is:

```text
(theta_base, theta1, theta2, theta3, phi)
```

This helps reduce unnecessary joint movement and provides a practical way to choose between multiple valid IK solutions.

---

## Visualization

The project contains two separate visualization modes.

### 1. Joint Angle Visualization

The joint visualization allows the user to control the robot directly using sliders.

The interface provides sliders for:

```text
Base
Theta 1
Theta 2
Theta 3
```

Moving a slider immediately updates the arm using forward kinematics.

The visualization displays:

* 3D robotic arm
* Base position
* Joint positions
* End-effector position
* Current joint angles
* End-effector orientation

This mode is useful for testing forward kinematics and understanding how each joint affects the arm.

---

### 2. Inverse Kinematics Visualization

The IK visualization allows the user to enter a desired endpoint:

```text
X
Y
Z
```

The user can then press:

```text
Solve IK
```

The visualization calls the inverse kinematics solver and updates the robot to the selected solution.

The interface displays:

* Target position
* Calculated joint angles
* Calculated `phi`
* Actual end-effector position
* Position error
* IK solver status

The target point is displayed in the 3D workspace so the user can visually compare the desired and actual endpoint.

---

## Current GUI Layout

The inverse kinematics interface is divided into two main sections.

### 3D Workspace

The left side displays:

```text
           End Effector
                ●
               /
              /
             ●
            /
           ●
          /
         ●
        Base
```

The coordinate system displays:

```text
X axis
Y axis
Z axis
```

### Control Panel

The right side contains:

```text
TARGET POSITION

X [          ]
Y [          ]
Z [          ]

[ Solve IK ] [ Clear ]

IK SOLUTION

Base
Theta1
Theta2
Theta3
Phi

END EFFECTOR

Target
Actual
Error

STATUS
```

This layout keeps the controls and information separate from the 3D workspace.

---

## Project Structure

```text
4-DOF-Arm/
│
├── config.py
├── main.py
│
├── Kinematics/
│   ├── __init__.py
│   └── kinematics.py
│
└── Visualization/
    ├── __init__.py
    ├── joint_visualizer.py
    └── ik_visualizer.py
```

### `config.py`

Contains the robot dimensions and joint limits.

### `main.py`

Creates the kinematics object and allows the user to select between the two visualization modes.

### `Kinematics/kinematics.py`

Contains the mathematical model for:

* Forward kinematics
* Joint positions
* Joint limit validation
* Inverse kinematics
* Phi search
* IK solution selection

### `Visualization/joint_visualizer.py`

Contains the interactive slider-based forward kinematics visualization.

### `Visualization/ik_visualizer.py`

Contains the endpoint-based inverse kinematics visualization.

### `__init__.py`

Exposes the classes as Python packages so they can be imported cleanly:

```python
from Kinematics import Kinematics
from Visualization import JointVisualizer, IKVisualizer
```

---

## Running the Project

From the project directory:

```bash
cd /home/jr/Projects/Robotics/4-DOF-Arm
```

Run:

```bash
python3 main.py
```

The terminal will display:

```text
==============================
      4-DOF ROBOTIC ARM
==============================

Select visualization:

1. Joint Angle Control
2. Inverse Kinematics
0. Exit
```

Select:

```text
1
```

for the slider-based forward kinematics visualization.

Select:

```text
2
```

for the endpoint-based inverse kinematics visualization.

---

## Example IK Test

A target such as:

```text
X = 1.0
Y = 0.0
Z = 1.0
```

can be entered into the IK visualization.

The solver searches for a valid combination of:

```text
theta_base
theta1
theta2
theta3
phi
```

that reaches the requested position while respecting the configured joint limits.

The resulting endpoint is then compared with the target and the position error is displayed.

---

## Design Approach

The project intentionally separates responsibilities.

### Kinematics

Responsible only for:

```text
Robot geometry
Joint limits
Forward kinematics
Inverse kinematics
Solution selection
```

### Visualization

Responsible only for:

```text
User input
Sliders
Buttons
3D rendering
Displaying results
```

### Main Program

Responsible for:

```text
Creating the kinematics model
Selecting the visualization mode
Starting the selected interface
```

This separation makes the project easier to test, debug, and extend.

---

## Current Limitations

The current IK solver uses a numerical search over `phi` rather than solving directly for a unique end-effector orientation.

The default search uses:

```text
phi step = 1°
```

This makes the solver straightforward to understand, but increasing the resolution also increases the number of candidate configurations that must be evaluated.

The current implementation also focuses on position rather than full end-effector orientation control.

---

## Future Improvements

Potential future improvements include:
* Add a physical robotic arm implementation.
* Connect joint angles to real motors.
* Add a 4-DOF mechanical design and CAD model.
* Add unit tests for forward and inverse kinematics.

---

## Technologies Used

* Python
* NumPy
* Matplotlib
* Matplotlib Widgets

The project is intended as a learning and development platform for robotic arm kinematics and serves as a foundation for eventually moving the controller to physical hardware.
