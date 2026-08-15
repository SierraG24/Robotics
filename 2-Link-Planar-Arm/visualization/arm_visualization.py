import numpy as np
import matplotlib.pyplot as plt

from kinematics.kinematics import (
    L1,
    L2,
    forward_kinematics,
    inverse_kinematics
)

from config import (
    THETA1_MIN,
    THETA1_MAX,
    THETA2_MIN,
    THETA2_MAX
)


class ArmVisualizer:

    def __init__(self):

        self.fig, self.ax = plt.subplots()

        # --------------------------------------------------
        # Current target
        # --------------------------------------------------

        self.target = None

        # --------------------------------------------------
        # Current joint angles
        # --------------------------------------------------

        self.theta1 = 0.0
        self.theta2 = 0.0

        # --------------------------------------------------
        # Create arm lines
        # --------------------------------------------------

        self.link1, = self.ax.plot(
            [],
            [],
            "o-",
            linewidth=4,
            markersize=8,
            label="Link 1"
        )

        self.link2, = self.ax.plot(
            [],
            [],
            "o-",
            linewidth=4,
            markersize=8,
            label="Link 2"
        )

        # --------------------------------------------------
        # Target point
        # --------------------------------------------------

        self.target_point, = self.ax.plot(
            [],
            [],
            "rx",
            markersize=12,
            markeredgewidth=3,
            label="Target"
        )

        # --------------------------------------------------
        # End effector
        # --------------------------------------------------

        self.end_effector, = self.ax.plot(
            [],
            [],
            "ko",
            markersize=8,
            label="End Effector"
        )

        # --------------------------------------------------
        # Calculate workspace
        # --------------------------------------------------

        self.workspace_x, self.workspace_y = (
            self.calculate_workspace()
        )

        self.setup_plot()

        # --------------------------------------------------
        # Mouse callback
        # --------------------------------------------------

        self.fig.canvas.mpl_connect(
            "button_press_event",
            self.on_click
        )

    # ======================================================
    # WORKSPACE
    # ======================================================

    def calculate_workspace(self):
        """
        Calculate the actual reachable workspace based
        on the limits of theta1 and theta2.
        """

        # Number of samples for each joint
        theta1_samples = np.linspace(
            THETA1_MIN,
            THETA1_MAX,
            300
        )

        theta2_samples = np.linspace(
            THETA2_MIN,
            THETA2_MAX,
            300
        )

        # Create all combinations of joint angles
        theta1_grid, theta2_grid = np.meshgrid(
            theta1_samples,
            theta2_samples
        )

        # Calculate end-effector position
        x = (
            L1 * np.cos(theta1_grid)
            + L2 * np.cos(
                theta1_grid + theta2_grid
            )
        )

        y = (
            L1 * np.sin(theta1_grid)
            + L2 * np.sin(
                theta1_grid + theta2_grid
            )
        )

        return x.flatten(), y.flatten()

    # ======================================================
    # PLOT SETUP
    # ======================================================

    def setup_plot(self):
        """Configure the Matplotlib window."""

        max_reach = L1 + L2

        # --------------------------------------------------
        # Plot limits
        # --------------------------------------------------

        self.ax.set_xlim(
            -max_reach - 0.25,
            max_reach + 0.25
        )

        self.ax.set_ylim(
            -0.25,
            max_reach + 0.25
        )

        self.ax.set_aspect("equal")

        # --------------------------------------------------
        # Labels
        # --------------------------------------------------

        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")

        self.ax.set_title(
            "2-Link Robotic Arm\n"
            "Click anywhere to select a target"
        )

        self.ax.grid(True)

        # --------------------------------------------------
        # X-axis
        # --------------------------------------------------

        self.ax.axhline(
            y=0,
            linewidth=1
        )

        # --------------------------------------------------
        # Plot reachable workspace
        # --------------------------------------------------

        self.ax.scatter(
            self.workspace_x,
            self.workspace_y,
            s=1,
            alpha=0.08,
            label="Reachable Workspace"
        )

        # --------------------------------------------------
        # Theta 1 = 0°
        # --------------------------------------------------

        self.ax.plot(
            [0, L1],
            [0, 0],
            linestyle="--",
            linewidth=1
        )

        # --------------------------------------------------
        # Theta 1 = 180°
        # --------------------------------------------------

        self.ax.plot(
            [0, -L1],
            [0, 0],
            linestyle="--",
            linewidth=1
        )

        # --------------------------------------------------
        # Base
        # --------------------------------------------------

        self.ax.plot(
            0,
            0,
            "ks",
            markersize=10,
            label="Base"
        )

        self.ax.legend()

        self.update_title()

    # ======================================================
    # MOUSE CLICK
    # ======================================================

    def on_click(self, event):

        if event.inaxes != self.ax:
            return

        x = event.xdata
        y = event.ydata

        # Store target
        self.target = (x, y)

        # Solve target
        self.solve_target()

    # ======================================================
    # UPDATE ARM
    # ======================================================

    def update_arm(self):
        """Update the arm using the current joint angles."""

        # --------------------------------------------------
        # Calculate elbow position
        # --------------------------------------------------

        elbow_x = L1 * np.cos(self.theta1)
        elbow_y = L1 * np.sin(self.theta1)

        # --------------------------------------------------
        # Calculate end-effector position
        # --------------------------------------------------

        end_x, end_y = forward_kinematics(
            self.theta1,
            self.theta2
        )

        # --------------------------------------------------
        # First link
        # --------------------------------------------------

        self.link1.set_data(
            [0, elbow_x],
            [0, elbow_y]
        )

        # --------------------------------------------------
        # Second link
        # --------------------------------------------------

        self.link2.set_data(
            [elbow_x, end_x],
            [elbow_y, end_y]
        )

        # --------------------------------------------------
        # End effector
        # --------------------------------------------------

        self.end_effector.set_data(
            [end_x],
            [end_y]
        )

        # --------------------------------------------------
        # Target
        # --------------------------------------------------

        if self.target is not None:

            self.target_point.set_data(
                [self.target[0]],
                [self.target[1]]
            )

        self.fig.canvas.draw_idle()

    # ======================================================
    # TITLE
    # ======================================================

    def update_title(self):

        self.ax.set_title(
            "2-Link Robotic Arm\n"
            "Click anywhere to select a target"
        )

        self.fig.canvas.draw_idle()

    # ======================================================
    # SOLVE TARGET
    # ======================================================

    def solve_target(self):

        if self.target is None:
            return

        x, y = self.target

        print(
            f"\nTarget: ({x:.3f}, {y:.3f})"
        )

        try:

            result = inverse_kinematics(
                x,
                y,
                self.theta1,
                self.theta2
            )

            # --------------------------------------------------
            # No valid IK solution
            # --------------------------------------------------

            if result is None:

                print(
                    "Target is inside the mathematical "
                    "workspace but cannot be reached "
                    "with the current joint limits."
                )

                return

            # --------------------------------------------------
            # Store joint angles
            # --------------------------------------------------

            self.theta1, self.theta2 = result

            # --------------------------------------------------
            # Print angles
            # --------------------------------------------------

            print(
                f"Theta 1: "
                f"{np.degrees(self.theta1):.2f}°"
            )

            print(
                f"Theta 2: "
                f"{np.degrees(self.theta2):.2f}°"
            )

            # --------------------------------------------------
            # Update visualization
            # --------------------------------------------------

            self.update_arm()

        except ValueError as error:

            print(
                f"Invalid target: {error}"
            )

    # ======================================================
    # SHOW
    # ======================================================

    def show(self):
        """Display the visualization."""

        plt.show()


# ==========================================================
# MAIN
# ==========================================================

def main():

    visualizer = ArmVisualizer()

    visualizer.show()


if __name__ == "__main__":
    main()