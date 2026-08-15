from config import (
    L1,
    L2,
    L3,
    PHI,
    THETA1_MIN,
    THETA_MIN,
    THETA_MAX
)

from kinematics import Kinematics
from visualization import ArmVisualizer


def main():

    # ==========================================================
    # IK MODE
    # ==========================================================

    # Options:
    # "locked" -> use fixed PHI
    # "search" -> search for the best PHI
    IK_MODE = "search"

    # ==========================================================
    # Create kinematics model
    # ==========================================================

    kin = Kinematics(
        L1=L1,
        L2=L2,
        L3=L3,
        PHI=PHI,
        theta1_min=THETA1_MIN,
        theta_min=THETA_MIN,
        theta_max=THETA_MAX
    )

    # ==========================================================
    # Initial robot configuration
    # ==========================================================

    kin.setCurrentAngles(
        0.0,
        0.0,
        0.0
    )

    # ==========================================================
    # Create visualizer
    # ==========================================================

    visualizer = ArmVisualizer(
        kin,
        ik_mode=IK_MODE,
        phi_min=0.0,
        phi_max=360.0,
        phi_step=1.0
    )

    # ==========================================================
    # Start GUI
    # ==========================================================

    visualizer.show()


if __name__ == "__main__":
    main()