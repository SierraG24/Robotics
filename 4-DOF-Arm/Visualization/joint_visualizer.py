import numpy as np
import matplotlib.pyplot as plt

from matplotlib.widgets import Slider


class JointVisualizer:
    """
    3D visualization of the 4-DOF robotic arm using
    joint-angle sliders.

    The user directly controls:

        theta_base
        theta1
        theta2
        theta3

    The arm is updated using forward kinematics.
    """

    def __init__(self, kinematics):

        self.kinematics = kinematics

        # --------------------------------------------------
        # Figure
        # --------------------------------------------------

        self.fig = plt.figure(figsize=(12, 8))

        self.ax = self.fig.add_subplot(
            111,
            projection="3d"
        )

        self.fig.subplots_adjust(
            left=0.05,
            right=0.75,
            bottom=0.25
        )

        # --------------------------------------------------
        # Current configuration
        # --------------------------------------------------

        (
            self.theta_base,
            self.theta1,
            self.theta2,
            self.theta3
        ) = self.kinematics.getCurrentAngles()

        # --------------------------------------------------
        # Workspace
        # --------------------------------------------------

        self.workspace_radius = (
            self.kinematics.L1 +
            self.kinematics.L2 +
            self.kinematics.L3
        )

        # --------------------------------------------------
        # Setup
        # --------------------------------------------------

        self._setupAxes()
        self._drawBase()
        self._createArm()
        self._createInfoText()
        self._createSliders()

        # Initial arm
        self.updateArm()

    # ======================================================
    # AXES
    # ======================================================

    def _setupAxes(self):

        R = self.workspace_radius * 1.15

        self.ax.set_xlim(-R, R)
        self.ax.set_ylim(-R, R)
        self.ax.set_zlim(0, R)

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        self.ax.set_title(
            "4-DOF Robotic Arm - Joint Control"
        )

        self.ax.set_box_aspect(
            (1, 1, 1)
        )

    # ======================================================
    # BASE
    # ======================================================

    def _drawBase(self):

        R = self.workspace_radius

        self.ax.scatter(
            [0],
            [0],
            [0],
            s=100
        )

        # X axis
        self.ax.plot(
            [0, R],
            [0, 0],
            [0, 0],
            linestyle="--"
        )

        # Y axis
        self.ax.plot(
            [0, 0],
            [0, R],
            [0, 0],
            linestyle="--"
        )

        # Z axis
        self.ax.plot(
            [0, 0],
            [0, 0],
            [0, R],
            linestyle="--"
        )

    # ======================================================
    # ARM
    # ======================================================

    def _createArm(self):

        self.arm_line, = self.ax.plot(
            [],
            [],
            [],
            marker="o",
            linewidth=4,
            markersize=8
        )

        self.end_effector, = self.ax.plot(
            [],
            [],
            [],
            marker="o",
            markersize=10
        )

    # ======================================================
    # INFO
    # ======================================================

    def _createInfoText(self):

        self.info_text = self.fig.text(
            0.78,
            0.75,
            "",
            fontsize=10,
            verticalalignment="top"
        )

    # ======================================================
    # SLIDERS
    # ======================================================

    def _createSliders(self):

        slider_left = 0.10
        slider_width = 0.55
        slider_height = 0.03

        # --------------------------------------------------
        # Base
        # --------------------------------------------------

        ax_base = self.fig.add_axes(
            [
                slider_left,
                0.17,
                slider_width,
                slider_height
            ]
        )

        self.slider_base = Slider(
            ax_base,
            "Base",
            np.degrees(
                self.kinematics.theta_base_min
            ),
            np.degrees(
                self.kinematics.theta_base_max
            ),
            valinit=np.degrees(
                self.theta_base
            ),
            valfmt="%.1f°"
        )

        # --------------------------------------------------
        # Theta 1
        # --------------------------------------------------

        ax_theta1 = self.fig.add_axes(
            [
                slider_left,
                0.13,
                slider_width,
                slider_height
            ]
        )

        self.slider_theta1 = Slider(
            ax_theta1,
            "Theta 1",
            np.degrees(
                self.kinematics.theta1_min
            ),
            np.degrees(
                self.kinematics.theta1_max
            ),
            valinit=np.degrees(
                self.theta1
            ),
            valfmt="%.1f°"
        )

        # --------------------------------------------------
        # Theta 2
        # --------------------------------------------------

        ax_theta2 = self.fig.add_axes(
            [
                slider_left,
                0.09,
                slider_width,
                slider_height
            ]
        )

        self.slider_theta2 = Slider(
            ax_theta2,
            "Theta 2",
            np.degrees(
                self.kinematics.theta2_min
            ),
            np.degrees(
                self.kinematics.theta2_max
            ),
            valinit=np.degrees(
                self.theta2
            ),
            valfmt="%.1f°"
        )

        # --------------------------------------------------
        # Theta 3
        # --------------------------------------------------

        ax_theta3 = self.fig.add_axes(
            [
                slider_left,
                0.05,
                slider_width,
                slider_height
            ]
        )

        self.slider_theta3 = Slider(
            ax_theta3,
            "Theta 3",
            np.degrees(
                self.kinematics.theta3_min
            ),
            np.degrees(
                self.kinematics.theta3_max
            ),
            valinit=np.degrees(
                self.theta3
            ),
            valfmt="%.1f°"
        )

        # --------------------------------------------------
        # Callbacks
        # --------------------------------------------------

        self.slider_base.on_changed(
            self.updateFromSliders
        )

        self.slider_theta1.on_changed(
            self.updateFromSliders
        )

        self.slider_theta2.on_changed(
            self.updateFromSliders
        )

        self.slider_theta3.on_changed(
            self.updateFromSliders
        )

    # ======================================================
    # SLIDER UPDATE
    # ======================================================

    def updateFromSliders(self, value=None):

        self.theta_base = np.radians(
            self.slider_base.val
        )

        self.theta1 = np.radians(
            self.slider_theta1.val
        )

        self.theta2 = np.radians(
            self.slider_theta2.val
        )

        self.theta3 = np.radians(
            self.slider_theta3.val
        )

        try:

            self.kinematics.setCurrentAngles(
                self.theta_base,
                self.theta1,
                self.theta2,
                self.theta3
            )

            self.updateArm()

        except ValueError as error:

            print(
                f"Joint limit error: {error}"
            )

    # ======================================================
    # UPDATE ARM
    # ======================================================

    def updateArm(self):

        positions = (
            self.kinematics.getJointPositions(
                self.theta_base,
                self.theta1,
                self.theta2,
                self.theta3
            )
        )

        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]

        # Arm
        self.arm_line.set_data(
            xs,
            ys
        )

        self.arm_line.set_3d_properties(
            zs
        )

        # End effector
        self.end_effector.set_data(
            [xs[-1]],
            [ys[-1]]
        )

        self.end_effector.set_3d_properties(
            [zs[-1]]
        )

        # End-effector orientation
        phi = (
            self.theta1 +
            self.theta2 +
            self.theta3
        )

        info = (
            "CURRENT CONFIGURATION\n"
            "\n"
            f"Base:   "
            f"{np.degrees(self.theta_base):7.2f}°\n"
            f"Theta1: "
            f"{np.degrees(self.theta1):7.2f}°\n"
            f"Theta2: "
            f"{np.degrees(self.theta2):7.2f}°\n"
            f"Theta3: "
            f"{np.degrees(self.theta3):7.2f}°\n"
            f"Phi:    "
            f"{np.degrees(phi):7.2f}°\n"
            "\n"
            "END EFFECTOR\n"
            "\n"
            f"X: {xs[-1]: .3f}\n"
            f"Y: {ys[-1]: .3f}\n"
            f"Z: {zs[-1]: .3f}"
        )

        self.info_text.set_text(info)

        self.fig.canvas.draw_idle()

    # ======================================================
    # SHOW
    # ======================================================

    def show(self):

        plt.show()