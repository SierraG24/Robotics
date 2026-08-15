import numpy as np

from controller.vehicle_controller import (
    VehicleController
)

from tuning.parameter_sweep import (
    load_pid_table,
    run_parameter_sweep,
    print_parameter_results,
    get_best_result
)

from visualization.visualizer import (
    plot_results
)


def select_mode():
    """
    Ask the user which PID mode to use.
    """

    print()
    print("PID LINE FOLLOWER")
    print("=================")
    print()
    print("1. Enter PID values manually")
    print("2. Read PID values from table")
    print("3. Read preset PID values")

    choice = input(
        "\nSelect mode (1-3): "
    )

    if choice == "1":

        return "manual"

    elif choice == "2":

        return "table"
    elif choice == "3":
        return "preset"

    else:

        print(
            "Invalid choice. "
            "Using manual mode."
        )

        return "manual"


def select_track():

    print()
    print("Select a track:")
    print("1. Straight line")
    print("2. Sine wave")
    print("3. Square wave")
    print("4. Trapezoid")
    print("5. Smooth trapezoid")
    print("6. Triangle")
    print("7. Circle")
    print("8. Oval")
    print("9. S-curve")
    print("10. Zigzag")
    print("11. Figure 8")

    choice = input(
        "Enter your choice (1-11): "
    )

    track_map = {

        "1": "straight",
        "2": "sine",
        "3": "square",
        "4": "trapezoid",
        "5": "smooth_trapezoid",
        "6": "triangle",
        "7": "circle",
        "8": "oval",
        "9": "s_curve",
        "10": "zigzag",
        "11": "figure8"
    }

    if choice not in track_map:

        print(
            "Invalid choice. "
            "Using straight line."
        )

        return "straight"

    return track_map[choice]

def enter_pid_values():
    """
    Ask the user to enter Kp, Ki, and Kd.
    """

    print()
    print("Enter PID values")
    print("----------------")

    kp = float(
        input("Kp: ")
    )

    ki = float(
        input("Ki: ")
    )

    kd = float(
        input("Kd: ")
    )

    return {
        "kp": kp,
        "ki": ki,
        "kd": kd
    }


def create_controller_params(pid, base_params):
    """
    Combine PID values with simulation parameters.
    """

    return {

        "kp": pid["kp"],

        "ki": pid["ki"],

        "kd": pid["kd"],

        "max_steering": (
            base_params[
                "max_steering"
            ]
        ),

        "max_steering_rate": (
            base_params[
                "max_steering_rate"
            ]
        ),

        "integral_limit": (
            base_params[
                "integral_limit"
            ]
        )
    }


def run_manual_mode(
    track_type,
    base_params
):
    """
    Run one manually selected PID controller.
    """

    pid = enter_pid_values()

    controller_params = (
        create_controller_params(
            pid,
            base_params
        )
    )

    controller = VehicleController(

        controller_params,

        dt=base_params["dt"],

        simulation_time=(
            base_params[
                "simulation_time"
            ]
        ),

        velocity=(
            base_params[
                "velocity"
            ]
        ),

        wheelbase=(
            base_params[
                "wheelbase"
            ]
        )
    )

    results = controller.simulate(
        track_type
    )

    print()
    print("==============================")
    print("PID RESULTS")
    print("==============================")

    print(
        f"Kp   = {pid['kp']}"
    )

    print(
        f"Ki   = {pid['ki']}"
    )

    print(
        f"Kd   = {pid['kd']}"
    )

    print(
        f"RMSE = {results['rmse']:.6f} m"
    )

    return results

def run_preset_mode(
    track_type,
    base_params
):
    """
    Run preselected PID controller.
    """

    pid = {
        "kp": 35.625,
        "ki": 0.001,
        "kd": 78.125
    }

    controller_params = (
        create_controller_params(
            pid,
            base_params
        )
    )

    controller = VehicleController(

        controller_params,

        dt=base_params["dt"],

        simulation_time=(
            base_params[
                "simulation_time"
            ]
        ),

        velocity=(
            base_params[
                "velocity"
            ]
        ),

        wheelbase=(
            base_params[
                "wheelbase"
            ]
        )
    )

    results = controller.simulate(
        track_type
    )

    print()
    print("==============================")
    print("PID VALUES")
    print("==============================")

    print(
        f"Kp   = {pid['kp']}"
    )

    print(
        f"Ki   = {pid['ki']}"
    )

    print(
        f"Kd   = {pid['kd']}"
    )

    print(
        f"RMSE = {results['rmse']:.6f} m"
    )

    return results


def run_table_mode(
    track_type,
    base_params
):
    """
    Load PID values from CSV, run every
    controller, and return the best result.
    """

    filename = input(
        "\nEnter PID table filename "
        "[data/pid_values.csv]: "
    )

    if filename.strip() == "":
        filename = "data/pid_values.csv"

    # --------------------------------------------
    # LOAD TABLE
    # --------------------------------------------

    pid_values = load_pid_table(
        filename
    )

    print()
    print(
        f"Loaded {len(pid_values)} "
        f"PID combinations."
    )

    # --------------------------------------------
    # RUN ALL CONTROLLERS
    # --------------------------------------------

    results = run_parameter_sweep(

        pid_values,

        track_type,

        base_params
    )

    # --------------------------------------------
    # PRINT RESULTS
    # --------------------------------------------

    print_parameter_results(
        results
    )

    # --------------------------------------------
    # GET BEST CONTROLLER
    # --------------------------------------------

    best_result = get_best_result(
        results
    )

    params = best_result[
        "controller_params"
    ]

    print()
    print("==============================")
    print("BEST PID CONTROLLER")
    print("==============================")

    print(
        f"Kp   = {params['kp']}"
    )

    print(
        f"Ki   = {params['ki']}"
    )

    print(
        f"Kd   = {params['kd']}"
    )

    print(
        f"RMSE = {best_result['rmse']:.5f} m"
    )

    return best_result


def main():

    # ==================================================
    # SIMULATION PARAMETERS
    # ==================================================

    base_params = {

        "max_steering": np.radians(35),

        "max_steering_rate": np.radians(120),

        "integral_limit": 1.5,

        "dt": 0.01,

        "simulation_time": 32.0,

        "velocity": 0.75,

        "wheelbase": 0.8
    }

    # ==================================================
    # SELECT MODE
    # ==================================================

    mode = select_mode()

    # ==================================================
    # SELECT TRACK
    # ==================================================

    track_type = select_track()

    # ==================================================
    # RUN SELECTED MODE
    # ==================================================

    if mode == "manual":
        results = run_manual_mode(
            track_type,
            base_params
        )

    elif mode == "preset":
        results = run_preset_mode(
            track_type, base_params
        )

    else:

        results = run_table_mode(
            track_type,
            base_params
        )

    # ==================================================
    # ANIMATE RESULT
    # ==================================================

    plot_results(
        results
    )


if __name__ == "__main__":
    main()