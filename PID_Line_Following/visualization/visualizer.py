import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def draw_car(
    ax,
    x,
    y,
    theta
):
    """
    Draw the vehicle and heading.
    """

    length = 1.0
    width = 0.5

    corners = np.array([
        [-length / 2, -width / 2],
        [length / 2, -width / 2],
        [length / 2, width / 2],
        [-length / 2, width / 2]
    ])

    rotation_matrix = np.array([
        [
            np.cos(theta),
            -np.sin(theta)
        ],
        [
            np.sin(theta),
            np.cos(theta)
        ]
    ])

    rotated_corners = (
        corners
        @ rotation_matrix.T
    )

    rotated_corners[:, 0] += x
    rotated_corners[:, 1] += y

    rotated_corners = np.vstack([
        rotated_corners,
        rotated_corners[0]
    ])

    car_line, = ax.plot(
        rotated_corners[:, 0],
        rotated_corners[:, 1],
        linewidth=3
    )

    heading_length = 0.7

    heading_line, = ax.plot(
        [
            x,
            x + heading_length * np.cos(theta)
        ],
        [
            y,
            y + heading_length * np.sin(theta)
        ],
        linewidth=2
    )

    return car_line, heading_line


def dt_to_ms(results):
    """
    Convert simulation timestep to milliseconds.
    """

    if len(results["time"]) > 1:

        dt = (
            results["time"][1]
            -
            results["time"][0]
        )

        return dt * 1000 * 3

    return 50


def plot_results(results):
    """
    Display simulation results and animation.
    """

    # ====================================================
    # GET RESULTS
    # ====================================================

    path_x = results["path_x"]
    path_y = results["path_y"]

    robot_x = results["robot_x"]
    robot_y = results["robot_y"]

    robot_theta = results["robot_theta"]

    error = results["error"]
    time = results["time"]

    rmse = results["rmse"]

    controller_params = (
        results["controller_params"]
    )

    kp = controller_params["kp"]
    ki = controller_params["ki"]
    kd = controller_params["kd"]

    max_steering = (
        controller_params["max_steering"]
    )

    integral_limit = (
        controller_params["integral_limit"]
    )

    # ====================================================
    # CREATE FIGURE
    # ====================================================

    fig = plt.figure(
        figsize=(12, 8)
    )

    # ====================================================
    # TRACK
    # ====================================================

    ax_track = fig.add_subplot(
        2,
        1,
        1
    )

    ax_track.plot(
        path_x,
        path_y,
        "--",
        linewidth=2,
        label="Reference Track"
    )

    trajectory_line, = ax_track.plot(
        [],
        [],
        linewidth=2,
        label="Car Trajectory"
    )

    car_line, = ax_track.plot(
        [],
        [],
        linewidth=3
    )

    heading_line, = ax_track.plot(
        [],
        [],
        linewidth=2
    )

    ax_track.set_title(
        "PID Line Follower"
    )

    ax_track.set_xlabel(
        "X Position (m)"
    )

    ax_track.set_ylabel(
        "Y Position (m)"
    )

    ax_track.axis("equal")
    ax_track.grid(True)
    ax_track.legend()

    # ====================================================
    # INFORMATION BOX
    # ====================================================

    info_text = ax_track.text(

        0.02,

        0.98,

        "",

        transform=ax_track.transAxes,

        verticalalignment="top",

        horizontalalignment="left",

        fontsize=8,

        family="monospace",

        bbox=dict(
            boxstyle="round",
            facecolor="white",
            alpha=0.8
        )
    )

    # ====================================================
    # ERROR PLOT
    # ====================================================

    ax_error = fig.add_subplot(
        2,
        1,
        2
    )

    ax_error.set_title(
        "Cross-Track Error"
    )

    ax_error.set_xlabel(
        "Time (s)"
    )

    ax_error.set_ylabel(
        "Error (m)"
    )

    ax_error.grid(True)

    if len(time) > 0:

        ax_error.set_xlim(
            time[0],
            time[-1]
        )

        error_margin = 0.2

        error_min = min(
            np.min(error),
            0
        )

        error_max = max(
            np.max(error),
            0
        )

        ax_error.set_ylim(
            error_min - error_margin,
            error_max + error_margin
        )

    error_line, = ax_error.plot(
        [],
        [],
        linewidth=2,
        label="Cross-Track Error"
    )

    ax_error.legend()

    # ====================================================
    # ANIMATION
    # ====================================================

    animation_step = 3

    frames = range(
        0,
        len(robot_x),
        animation_step
    )

    def update(frame):

        # -----------------------------------------------
        # CURRENT STATE
        # -----------------------------------------------

        x = robot_x[frame]
        y = robot_y[frame]
        theta = robot_theta[frame]

        # -----------------------------------------------
        # TRAJECTORY
        # -----------------------------------------------

        trajectory_line.set_data(
            robot_x[:frame + 1],
            robot_y[:frame + 1]
        )

        # -----------------------------------------------
        # CAR
        # -----------------------------------------------

        length = 1.0
        width = 0.5

        corners = np.array([
            [-length / 2, -width / 2],
            [length / 2, -width / 2],
            [length / 2, width / 2],
            [-length / 2, width / 2],
            [-length / 2, -width / 2]
        ])

        rotation_matrix = np.array([
            [
                np.cos(theta),
                -np.sin(theta)
            ],
            [
                np.sin(theta),
                np.cos(theta)
            ]
        ])

        rotated = (
            corners
            @ rotation_matrix.T
        )

        rotated[:, 0] += x
        rotated[:, 1] += y

        car_line.set_data(
            rotated[:, 0],
            rotated[:, 1]
        )

        # -----------------------------------------------
        # HEADING
        # -----------------------------------------------

        heading_length = 0.7

        heading_line.set_data(
            [
                x,
                x + heading_length * np.cos(theta)
            ],
            [
                y,
                y + heading_length * np.sin(theta)
            ]
        )

        # -----------------------------------------------
        # ERROR
        # -----------------------------------------------

        error_line.set_data(
            time[:frame + 1],
            error[:frame + 1]
        )

        # -----------------------------------------------
        # INFORMATION
        # -----------------------------------------------

        info_text.set_text(

            f"PID CONTROLLER\n"
            f"--------------------\n"
            f"Kp: {kp:.3f}\n"
            f"Ki: {ki:.3f}\n"
            f"Kd: {kd:.3f}\n"
            f"\n"
            f"RMSE: {rmse:.6f} m\n"
            f"\n"
            f"Max Steering:\n"
            f"{np.degrees(max_steering):.1f} deg\n"
            f"Integral Limit:\n"
            f"{integral_limit:.3f}"
        )

        return (
            trajectory_line,
            car_line,
            heading_line,
            error_line,
            info_text
        )

    animation = FuncAnimation(

        fig,

        update,

        frames=frames,

        interval=dt_to_ms(results),

        blit=True,

        repeat=False
    )

    plt.tight_layout()
    plt.show()

def print_parameter_results(results):
    """
    Print PID tuning results sorted by RMSE.
    """

    print("\n")
    print("=" * 65)
    print("PID PARAMETER RESULTS")
    print("=" * 65)

    print(
        f"{'Test':<8}"
        f"{'Kp':<12}"
        f"{'Ki':<12}"
        f"{'Kd':<12}"
        f"{'RMSE (m)':<12}"
    )

    print("-" * 65)

    for result in results:

        params = result[
            "controller_params"
        ]

        print(
            f"{result['test_number']:<8}"
            f"{params['kp']:<12.3f}"
            f"{params['ki']:<12.3f}"
            f"{params['kd']:<12.3f}"
            f"{result['rmse']:<12.5f}"
        )

    print("=" * 65)

    # Best controller
    best = results[0]

    params = best[
        "controller_params"
    ]

    print("\nBEST PID CONTROLLER")
    print("-------------------")

    print(
        f"Kp   = {params['kp']:.5f}"
    )

    print(
        f"Ki   = {params['ki']:.5f}"
    )

    print(
        f"Kd   = {params['kd']:.5f}"
    )

    print(
        f"RMSE = {best['rmse']:.7f} m"
    )