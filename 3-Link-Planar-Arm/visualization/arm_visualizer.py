import numpy as np
import matplotlib.pyplot as plt

from matplotlib.lines import Line2D
from matplotlib.patches import Circle


class ArmVisualizer:
    """
    Interactive visualization for a 3-link planar robotic arm.

    Left-click inside the plot to select an (x, y) target.

    IK modes:

        locked:
            Uses the fixed PHI value from the kinematics class.

        search:
            Searches through a range of phi values and selects
            the valid configuration closest to the current
            joint configuration.
    """

    def __init__(
        self,
        kinematics,
        ik_mode="locked",
        phi_min=0.0,
        phi_max=360.0,
        phi_step=1.0,
        workspace_scale=1.2
    ):

        self.kinematics = kinematics

        # ======================================================
        # IK settings
        # ======================================================

        if ik_mode not in ("locked", "search"):
            raise ValueError(
                "ik_mode must be 'locked' or 'search'"
            )

        self.ik_mode = ik_mode

        self.phi_min = phi_min
        self.phi_max = phi_max
        self.phi_step = phi_step

        self.workspace_scale = (
            workspace_scale
        )

        # Store the phi currently being used
        self.current_phi = (
            np.degrees(self.kinematics.PHI)
        )

        # ======================================================
        # Figure
        # ======================================================

        self.fig, self.ax = plt.subplots()

        # ======================================================
        # Robot
        # ======================================================

        self.robot_line = Line2D(
            [],
            [],
            marker="o",
            linewidth=3,
            markersize=8
        )

        self.ax.add_line(
            self.robot_line
        )

        # ======================================================
        # Target
        # ======================================================

        self.target_marker = Line2D(
            [],
            [],
            marker="x",
            markersize=12,
            markeredgewidth=3,
            linestyle="None"
        )

        self.ax.add_line(
            self.target_marker
        )

        # ======================================================
        # Mouse event
        # ======================================================

        self.fig.canvas.mpl_connect(
            "button_press_event",
            self._on_click
        )

        # ======================================================
        # Plot setup
        # ======================================================

        self._setup_plot()

        # ======================================================
        # Draw initial robot
        # ======================================================

        self.update()

    # ==========================================================
    # PLOT SETUP
    # ==========================================================

    def _setup_plot(self):

        max_reach = (
            self.kinematics.L1
            + self.kinematics.L2
            + self.kinematics.L3
        )

        limit = (
            max_reach
            * self.workspace_scale
        )

        self.ax.set_xlim(
            -limit,
            limit
        )

        self.ax.set_ylim(
            -limit,
            limit
        )

        self.ax.set_aspect(
            "equal",
            adjustable="box"
        )

        self.ax.axhline(
            0,
            linewidth=1
        )

        self.ax.axvline(
            0,
            linewidth=1
        )

        self.ax.grid(
            True,
            alpha=0.3
        )

        self.ax.set_xlabel(
            "X Position"
        )

        self.ax.set_ylabel(
            "Y Position"
        )

        self._update_title()

        self._draw_workspace()

    # ==========================================================
    # TITLE
    # ==========================================================

    def _update_title(self):

        if self.ik_mode == "locked":

            self.ax.set_title(
                f"3-Link Planar Arm | "
                f"IK: LOCKED | "
                f"φ = {np.degrees(self.kinematics.PHI):.1f}°"
            )

        else:

            self.ax.set_title(
                f"3-Link Planar Arm | "
                f"IK: SEARCH | "
                f"φ = {self.current_phi:.1f}°"
            )

    # ==========================================================
    # WORKSPACE
    # ==========================================================

    def _draw_workspace(self):

        L1 = self.kinematics.L1
        L2 = self.kinematics.L2
        L3 = self.kinematics.L3

        # ------------------------------------------------------
        # LOCKED PHI
        # ------------------------------------------------------

        if self.ik_mode == "locked":

            phi = self.kinematics.PHI

            r_outer = L1 + L2
            r_inner = abs(L1 - L2)

            center_x = (
                L3 * np.cos(phi)
            )

            center_y = (
                L3 * np.sin(phi)
            )

            outer_circle = Circle(
                (
                    center_x,
                    center_y
                ),
                r_outer,
                fill=True,
                alpha=0.08,
                linestyle="--",
                linewidth=1.5
            )

            self.ax.add_patch(
                outer_circle
            )

            outer_boundary = Circle(
                (
                    center_x,
                    center_y
                ),
                r_outer,
                fill=False,
                linestyle="--",
                linewidth=1.5
            )

            self.ax.add_patch(
                outer_boundary
            )

            if r_inner > 0:

                inner_boundary = Circle(
                    (
                        center_x,
                        center_y
                    ),
                    r_inner,
                    fill=False,
                    linestyle="--",
                    linewidth=1.5
                )

                self.ax.add_patch(
                    inner_boundary
                )

                inner_fill = Circle(
                    (
                        center_x,
                        center_y
                    ),
                    r_inner,
                    fill=True,
                    alpha=0.15
                )

                self.ax.add_patch(
                    inner_fill
                )

        # ------------------------------------------------------
        # SEARCH PHI
        # ------------------------------------------------------

        else:

            # When phi is free, the entire L3 link can rotate.
            #
            # Therefore the overall maximum reach is:
            #
            # L1 + L2 + L3
            #
            # and the minimum reach depends on the link lengths.

            max_reach = (
                L1 + L2 + L3
            )

            min_reach = max(
                0.0,
                max(
                    L1 - L2 - L3,
                    L2 - L1 - L3,
                    L3 - L1 - L2
                )
            )

            outer_boundary = Circle(
                (0, 0),
                max_reach,
                fill=False,
                linestyle="--",
                linewidth=1.5
            )

            self.ax.add_patch(
                outer_boundary
            )

            if min_reach > 0:

                inner_boundary = Circle(
                    (0, 0),
                    min_reach,
                    fill=False,
                    linestyle="--",
                    linewidth=1.5
                )

                self.ax.add_patch(
                    inner_boundary
                )

                inner_fill = Circle(
                    (0, 0),
                    min_reach,
                    fill=True,
                    alpha=0.15
                )

                self.ax.add_patch(
                    inner_fill
                )

    # ==========================================================
    # UPDATE
    # ==========================================================

    def update(self):
        """
        Update the robot drawing using the current
        kinematic configuration.
        """

        theta1, theta2, theta3 = (
            self.kinematics.getCurrentAngles()
        )

        positions = (
            self.kinematics.getJointPositions(
                theta1,
                theta2,
                theta3
            )
        )

        x = [
            position[0]
            for position in positions
        ]

        y = [
            position[1]
            for position in positions
        ]

        self.robot_line.set_data(
            x,
            y
        )

        self._update_title()

        self.fig.canvas.draw_idle()

    # ==========================================================
    # MOUSE INPUT
    # ==========================================================

    def _on_click(self, event):

        # Ignore click outside plot
        if event.inaxes != self.ax:
            return

        # Only left click
        if event.button != 1:
            return

        x = event.xdata
        y = event.ydata

        print()
        print(
            "=============================="
        )

        print(
            "Target:"
        )

        print(
            f"x = {x:.3f}"
        )

        print(
            f"y = {y:.3f}"
        )

        # Show target
        self.target_marker.set_data(
            [x],
            [y]
        )

        # ======================================================
        # LOCKED PHI
        # ======================================================

        if self.ik_mode == "locked":

            print()
            print(
                "IK Mode: LOCKED"
            )

            print(
                f"phi = "
                f"{np.degrees(self.kinematics.PHI):.2f}°"
            )

            try:

                solution = (
                    self.kinematics.phiLockedIK(
                        x,
                        y
                    )
                )

                theta1, theta2, theta3 = (
                    solution
                )

                self.current_phi = (
                    np.degrees(
                        self.kinematics.PHI
                    )
                )

                self._print_solution(
                    theta1,
                    theta2,
                    theta3
                )

                self.kinematics.setCurrentAngles(
                    theta1,
                    theta2,
                    theta3
                )

                self.update()

            except ValueError as error:

                print()
                print(
                    "Target unreachable:"
                )

                print(error)

        # ======================================================
        # SEARCH PHI
        # ======================================================

        else:

            print()
            print(
                "IK Mode: SEARCH"
            )

            print(
                f"Searching phi from "
                f"{self.phi_min:.1f}° to "
                f"{self.phi_max:.1f}° "
                f"in {self.phi_step:.1f}° steps"
            )

            try:

                solution = (
                    self.kinematics.findBestPhiValueIK(
                        x,
                        y,
                        phi_min=self.phi_min,
                        phi_max=self.phi_max,
                        phi_step=self.phi_step
                    )
                )

                theta1, theta2, theta3, phi = (
                    solution
                )

                self.current_phi = (
                    np.degrees(phi)
                )

                self._print_solution(
                    theta1,
                    theta2,
                    theta3
                )

                print(
                    f"phi = "
                    f"{np.degrees(phi):.2f}°"
                )

                self.kinematics.setCurrentAngles(
                    theta1,
                    theta2,
                    theta3
                )

                self.update()

            except ValueError as error:

                print()
                print(
                    "Target unreachable:"
                )

                print(error)

        print(
            "=============================="
        )

        self.fig.canvas.draw_idle()

    # ==========================================================
    # PRINT IK SOLUTION
    # ==========================================================

    def _print_solution(
        self,
        theta1,
        theta2,
        theta3
    ):

        print()
        print(
            "IK Solution:"
        )

        print(
            f"theta1 = "
            f"{np.degrees(theta1):.2f}°"
        )

        print(
            f"theta2 = "
            f"{np.degrees(theta2):.2f}°"
        )

        print(
            f"theta3 = "
            f"{np.degrees(theta3):.2f}°"
        )

    # ==========================================================
    # RUN
    # ==========================================================

    def show(self):

        plt.show()