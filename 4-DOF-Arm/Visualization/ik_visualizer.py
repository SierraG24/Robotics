import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox


class IKVisualizer:
    """
    3D inverse-kinematics visualization for the 4-DOF robotic arm.
    Shows only theta values in the GUI.
    """

    def __init__(self, kinematics):
        self.kinematics = kinematics

        # Figure
        self.fig = plt.figure(figsize=(14, 8))

        # 3D axes
        self.ax = self.fig.add_axes([0.05, 0.10, 0.62, 0.82], projection="3d")

        # Current angles
        (
            self.theta_base,
            self.theta1,
            self.theta2,
            self.theta3
        ) = self.kinematics.getCurrentAngles()

        self.phi = self.theta1 + self.theta2 + self.theta3

        # Workspace radius
        self.workspace_radius = (
            self.kinematics.L1 +
            self.kinematics.L2 +
            self.kinematics.L3
        )

        self.target = None

        self._setupAxes()
        self._drawBase()
        self._createArm()
        self._createTarget()
        self._createGUI()

        self.updateArm()

    # ======================================================
    # AXES SETUP
    # ======================================================

    def _setupAxes(self):
        R = self.workspace_radius * 1.15

        self.ax.set_xlim(-R, R)
        self.ax.set_ylim(-R, R)
        self.ax.set_zlim(0, R)

        self.ax.set_xlabel("X", labelpad=10)
        self.ax.set_ylabel("Y", labelpad=10)
        self.ax.set_zlabel("Z", labelpad=10)

        self.ax.set_title(
            "4-DOF Robotic Arm - Inverse Kinematics",
            pad=20,
            fontsize=14,
            fontweight="bold"
        )

        self.ax.set_box_aspect((1, 1, 1))
        self.ax.view_init(elev=25, azim=-60)

    # ======================================================
    # BASE
    # ======================================================

    def _drawBase(self):
        R = self.workspace_radius

        self.ax.scatter([0], [0], [0], s=100)

        self.ax.plot([0, R], [0, 0], [0, 0], linestyle="--")
        self.ax.plot([0, 0], [0, R], [0, 0], linestyle="--")
        self.ax.plot([0, 0], [0, 0], [0, R], linestyle="--")

    # ======================================================
    # ARM
    # ======================================================

    def _createArm(self):
        self.arm_line, = self.ax.plot([], [], [], marker="o", linewidth=5, markersize=8)
        self.end_effector, = self.ax.plot([], [], [], marker="o", markersize=12)

    # ======================================================
    # TARGET MARKER
    # ======================================================

    def _createTarget(self):
        self.target_point, = self.ax.plot(
            [], [], [],
            marker="x",
            markersize=15,
            markeredgewidth=3,
            linestyle="None"
        )

    # ======================================================
    # GUI (NON-OVERLAPPING + SAFE CALLBACKS)
    # ======================================================

    def _createGUI(self):
        left = 0.73
        width = 0.22

        # Title
        ax_title = self.fig.add_axes([left, 0.90, width, 0.05])
        ax_title.axis("off")
        ax_title.text(0, 1, "TARGET POSITION", fontsize=16, fontweight="bold", va="top")

        # X input
        ax_x = self.fig.add_axes([left, 0.83, width, 0.045])
        self.target_x = TextBox(ax_x, "X ", initial="1.0")

        # Y input
        ax_y = self.fig.add_axes([left, 0.76, width, 0.045])
        self.target_y = TextBox(ax_y, "Y ", initial="0.0")

        # Z input
        ax_z = self.fig.add_axes([left, 0.69, width, 0.045])
        self.target_z = TextBox(ax_z, "Z ", initial="1.0")

        # Solve button
        ax_solve = self.fig.add_axes([left, 0.61, 0.105, 0.055])
        self.solve_button = Button(ax_solve, "Solve IK")

        # Clear button
        ax_clear = self.fig.add_axes([left + 0.115, 0.61, 0.105, 0.055])
        self.clear_button = Button(ax_clear, "Clear")

        # IK Solution panel
        ax_solution = self.fig.add_axes([left, 0.42, width, 0.16])
        ax_solution.axis("off")
        ax_solution.text(0, 1, "IK SOLUTION", fontsize=15, fontweight="bold", va="top")

        self.solution_text = ax_solution.text(
            0, 0.85,
            "No solution calculated.",
            fontsize=11,
            va="top",
            family="monospace"
        )

        # STATUS panel
        ax_status = self.fig.add_axes([left, 0.20, width, 0.12])
        ax_status.axis("off")
        ax_status.text(0, 1, "STATUS", fontsize=15, fontweight="bold", va="top")

        self.status_text = ax_status.text(
            0, 0.75,
            "Ready.",
            fontsize=11,
            va="top",
            family="monospace"
        )

        # ======================================================
        # SAFE CALLBACKS (NO ResizeEvent CRASH)
        # ======================================================

        def _safe_solve(event):
            if hasattr(event, "inaxes") and event.inaxes == self.solve_button.ax:
                self.solveIK()

        def _safe_clear(event):
            if hasattr(event, "inaxes") and event.inaxes == self.clear_button.ax:
                self.clearTarget()

        self.fig.canvas.mpl_connect("button_press_event", _safe_solve)
        self.fig.canvas.mpl_connect("button_press_event", _safe_clear)

    # ======================================================
    # READ TARGET
    # ======================================================

    def _readTarget(self):
        try:
            x = float(self.target_x.text)
            y = float(self.target_y.text)
            z = float(self.target_z.text)
        except ValueError as error:
            raise ValueError("X, Y, and Z must be numbers.") from error
        return x, y, z

    # ======================================================
    # SOLVE IK
    # ======================================================

    def solveIK(self):
        try:
            x, y, z = self._readTarget()
        except ValueError as error:
            self.status_text.set_text(f"INPUT ERROR\n\n{error}")
            self.fig.canvas.draw_idle()
            return

        self.target = (x, y, z)

        self.target_point.set_data([x], [y])
        self.target_point.set_3d_properties([z])

        try:
            solution = self.kinematics.findBestPhiValueIK(x, y, z)
        except ValueError as error:
            self.solution_text.set_text("No valid IK solution.")
            self.status_text.set_text(f"IK FAILED\n\n{error}")
            self.fig.canvas.draw_idle()
            return

        (
            self.theta_base,
            self.theta1,
            self.theta2,
            self.theta3,
            self.phi
        ) = solution

        self.kinematics.setCurrentAngles(
            self.theta_base,
            self.theta1,
            self.theta2,
            self.theta3
        )

        self.updateArm()

        # Update solution text (theta values only)
        solution_text = (
            f"Base   = {np.degrees(self.theta_base):8.2f} deg\n"
            f"Theta1 = {np.degrees(self.theta1):8.2f} deg\n"
            f"Theta2 = {np.degrees(self.theta2):8.2f} deg\n"
            f"Theta3 = {np.degrees(self.theta3):8.2f} deg\n"
            f"Phi    = {np.degrees(self.phi):8.2f} deg"
        )

        self.solution_text.set_text(solution_text)
        self.status_text.set_text("SUCCESS")

        self.fig.canvas.draw_idle()

    # ======================================================
    # UPDATE ARM
    # ======================================================

    def updateArm(self):
        positions = self.kinematics.getJointPositions(
            self.theta_base,
            self.theta1,
            self.theta2,
            self.theta3
        )

        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]

        self.arm_line.set_data(xs, ys)
        self.arm_line.set_3d_properties(zs)

        self.end_effector.set_data([xs[-1]], [ys[-1]])
        self.end_effector.set_3d_properties([zs[-1]])

        self.fig.canvas.draw_idle()

    # ======================================================
    # CLEAR TARGET
    # ======================================================

    def clearTarget(self):
        self.target = None

        self.target_point.set_data([], [])
        self.target_point.set_3d_properties([])

        self.target_x.set_val("1.0")
        self.target_y.set_val("0.0")
        self.target_z.set_val("1.0")

        self.solution_text.set_text("No solution calculated.")
        self.status_text.set_text("Ready.")

        self.fig.canvas.draw_idle()

    # SHOW

    def show(self):
        plt.show()
