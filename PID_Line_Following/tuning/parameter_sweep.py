import csv

from controller.vehicle_controller import (
    VehicleController
)


def load_pid_table(filename):
    """
    Load PID values from a CSV file.

    CSV format:

        kp,ki,kd
        4.0,0.1,0.0
        5.0,0.2,0.1
        ...

    Returns
    -------
    list
        List of dictionaries containing Kp, Ki, and Kd.
    """

    pid_values = []

    with open(
        filename,
        "r",
        newline=""
    ) as file:

        reader = csv.DictReader(file)

        for row in reader:

            pid_values.append({

                "kp": float(row["kp"]),

                "ki": float(row["ki"]),

                "kd": float(row["kd"])
            })

    return pid_values


def run_parameter_sweep(
    pid_values,
    track_type,
    base_params
):
    """
    Run a simulation for every PID combination.

    Parameters
    ----------
    pid_values : list
        List of dictionaries containing kp, ki, kd.

    track_type : str
        Reference track.

    base_params : dict
        Vehicle and steering parameters.

    Returns
    -------
    list
        Simulation results sorted by RMSE.
    """

    all_results = []

    total_tests = len(pid_values)

    print()
    print(
        f"Running {total_tests} PID controllers..."
    )

    print(
        "----------------------------------------"
    )

    for test_number, pid in enumerate(
        pid_values,
        start=1
    ):

        kp = pid["kp"]
        ki = pid["ki"]
        kd = pid["kd"]

        print(
            f"Test {test_number}/{total_tests}: "
            f"Kp={kp}, "
            f"Ki={ki}, "
            f"Kd={kd}"
        )

        # --------------------------------------------
        # CONTROLLER PARAMETERS
        # --------------------------------------------

        controller_params = {

            "kp": kp,

            "ki": ki,

            "kd": kd,

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

        # --------------------------------------------
        # CREATE CONTROLLER
        # --------------------------------------------

        controller = VehicleController(

            controller_params,

            dt=base_params["dt"],

            simulation_time=(
                base_params[
                    "simulation_time"
                ]
            ),

            velocity=(
                base_params["velocity"]
            ),

            wheelbase=(
                base_params["wheelbase"]
            )
        )

        # --------------------------------------------
        # RUN SIMULATION
        # --------------------------------------------

        results = controller.simulate(
            track_type
        )

        # --------------------------------------------
        # ADD TEST NUMBER
        # --------------------------------------------

        results["test_number"] = (
            test_number
        )

        # --------------------------------------------
        # STORE RESULTS
        # --------------------------------------------

        all_results.append(
            results
        )

    # --------------------------------------------
    # SORT BY RMSE
    # --------------------------------------------

    all_results.sort(
        key=lambda result: result["rmse"]
    )

    return all_results


def print_parameter_results(results):
    """
    Print all PID results sorted by RMSE.
    """

    print()
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


def get_best_result(results):
    """
    Return the controller with the lowest RMSE.
    """

    if not results:

        raise ValueError(
            "No simulation results available."
        )

    return results[0]