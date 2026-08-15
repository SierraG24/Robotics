from Kinematics import Kinematics

from Visualization import (
    JointVisualizer,
    IKVisualizer
)

from config import (
    L1,
    L2,
    L3,
    THETA_BASE_MIN,
    THETA_BASE_MAX,
    THETA1_MIN,
    THETA2_MIN,
    THETA3_MIN,
    THETA1_MAX,
    THETA2_MAX,
    THETA3_MAX
)


def main():

    # ==================================================
    # Create kinematics
    # ==================================================

    kinematics = Kinematics(
        L1,
        L2,
        L3,
        THETA_BASE_MIN,
        THETA_BASE_MAX,
        THETA1_MIN,
        THETA2_MIN,
        THETA3_MIN,
        THETA1_MAX,
        THETA2_MAX,
        THETA3_MAX
    )

    # ==================================================
    # Select visualization
    # ==================================================

    print()
    print("==============================")
    print("      4-DOF ROBOTIC ARM")
    print("==============================")
    print()
    print("Select visualization:")
    print()
    print("1. Joint Angle Control")
    print("2. Inverse Kinematics")
    print("0. Exit")
    print()

    choice = input(
        "Enter selection: "
    ).strip()

    # ==================================================
    # Joint visualization
    # ==================================================

    if choice == "1":

        print()
        print("Starting Joint Angle Visualization...")
        print()

        visualizer = JointVisualizer(
            kinematics
        )

        visualizer.show()

    # ==================================================
    # IK visualization
    # ==================================================

    elif choice == "2":

        print()
        print("Starting Inverse Kinematics Visualization...")
        print()

        visualizer = IKVisualizer(
            kinematics
        )

        visualizer.show()

    # ==================================================
    # Exit
    # ==================================================

    elif choice == "0":

        print("Exiting...")

    else:

        print(
            "Invalid selection."
        )


if __name__ == "__main__":
    main()