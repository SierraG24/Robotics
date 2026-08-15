import numpy as np

from pid.pid_controller import PIDController


def create_reference_path(track_type):
    """
    Create a reference path.

    Parameters
    ----------
    track_type : str
        straight
        sine
        square
        trapezoid
        smooth_trapezoid
        triangle
        circle
        oval
        s_curve
        zigzag
        figure8

    Returns
    -------
    path_x, path_y : np.ndarray
        Reference path coordinates.
    """

    # ============================================================
    # STRAIGHT
    # ============================================================

    if track_type == "straight":

        x = np.linspace(
            0,
            20,
            1000
        )

        y = np.zeros_like(x)

    # ============================================================
    # SINE WAVE
    # ============================================================

    elif track_type == "sine":

        x = np.linspace(
            0,
            20,
            1000
        )

        y = (
            2.0
            * np.sin(0.5 * x)
        )

    # ============================================================
    # SQUARE WAVE
    # ============================================================

    elif track_type == "square":

        x = np.linspace(
            0,
            20,
            1000
        )

        y = (
            2.0
            * np.sign(
                np.sin(0.5 * x)
            )
        )

    # ============================================================
    # ORIGINAL TRAPEZOID
    # ============================================================

    elif track_type == "trapezoid":

        points = np.array([
            [0, 0],
            [5, 0],
            [7, 3],
            [12, 3],
            [14, 0],
            [20, 0]
        ])

        path_x = []
        path_y = []

        for i in range(
            len(points) - 1
        ):

            x_segment = np.linspace(
                points[i, 0],
                points[i + 1, 0],
                150
            )

            y_segment = np.linspace(
                points[i, 1],
                points[i + 1, 1],
                150
            )

            path_x.extend(
                x_segment
            )

            path_y.extend(
                y_segment
            )

        x = np.array(path_x)
        y = np.array(path_y)

    # ============================================================
    # SMOOTH TRAPEZOID
    #
    # Uses a smooth interpolation between points instead of
    # sharp corners.
    # ============================================================

    elif track_type == "smooth_trapezoid":

        points = np.array([
            [0, 0],
            [5, 0],
            [8, 3],
            [12, 3],
            [15, 0],
            [20, 0]
        ])

        path_x = []
        path_y = []

        for i in range(
            len(points) - 1
        ):

            x0 = points[i, 0]
            y0 = points[i, 1]

            x1 = points[i + 1, 0]
            y1 = points[i + 1, 1]

            t = np.linspace(
                0,
                1,
                200
            )

            # Smoothstep interpolation.
            #
            # This makes the transition between
            # segments less abrupt.

            smooth_t = (
                3 * t**2
                - 2 * t**3
            )

            x_segment = (
                x0
                + (x1 - x0)
                * smooth_t
            )

            y_segment = (
                y0
                + (y1 - y0)
                * smooth_t
            )

            path_x.extend(
                x_segment
            )

            path_y.extend(
                y_segment
            )

        x = np.array(path_x)
        y = np.array(path_y)

    # ============================================================
    # TRIANGLE
    # ============================================================

    elif track_type == "triangle":

        points = np.array([
            [0, 0],
            [10, 0],
            [5, 6],
            [0, 0]
        ])

        path_x = []
        path_y = []

        for i in range(
            len(points) - 1
        ):

            x_segment = np.linspace(
                points[i, 0],
                points[i + 1, 0],
                300
            )

            y_segment = np.linspace(
                points[i, 1],
                points[i + 1, 1],
                300
            )

            path_x.extend(
                x_segment
            )

            path_y.extend(
                y_segment
            )

        x = np.array(path_x)
        y = np.array(path_y)

    # ============================================================
    # CIRCLE
    # ============================================================

    elif track_type == "circle":

        radius = 5.0

        theta = np.linspace(
            0,
            2 * np.pi,
            1000
        )

        x = (
            radius
            * np.cos(theta)
        )

        y = (
            radius
            * np.sin(theta)
        )

    # ============================================================
    # OVAL
    # ============================================================

    elif track_type == "oval":

        theta = np.linspace(
            0,
            2 * np.pi,
            1200
        )

        x = (
            8.0
            * np.cos(theta)
        )

        y = (
            4.0
            * np.sin(theta)
        )

    # ============================================================
    # S-CURVE
    # ============================================================

    elif track_type == "s_curve":

        x = np.linspace(
            0,
            20,
            1200
        )

        y = (
            3.0
            * np.sin(
                0.35 * x
            )
        )

    # ============================================================
    # ZIGZAG
    # ============================================================

    elif track_type == "zigzag":

        points = np.array([
            [0, 0],
            [3, 3],
            [6, 0],
            [9, 3],
            [12, 0],
            [15, 3],
            [18, 0],
            [20, 0]
        ])

        path_x = []
        path_y = []

        for i in range(
            len(points) - 1
        ):

            x_segment = np.linspace(
                points[i, 0],
                points[i + 1, 0],
                120
            )

            y_segment = np.linspace(
                points[i, 1],
                points[i + 1, 1],
                120
            )

            path_x.extend(
                x_segment
            )

            path_y.extend(
                y_segment
            )

        x = np.array(path_x)
        y = np.array(path_y)

    # ============================================================
    # FIGURE 8
    # ============================================================

    elif track_type == "figure8":

        theta = np.linspace(
            0,
            2 * np.pi,
            1500
        )

        # Lemniscate-like figure 8

        x = (
            5.0
            * np.sin(theta)
        )

        y = (
            3.0
            * np.sin(theta)
            * np.cos(theta)
        )

    # ============================================================
    # UNKNOWN TRACK
    # ============================================================

    else:

        raise ValueError(
            f"Unknown track type: {track_type}"
        )

    return x, y

def find_nearest_point(
    robot_x,
    robot_y,
    path_x,
    path_y
):
    """
    Find the closest point on the reference path.
    """

    distances = np.sqrt(
        (path_x - robot_x) ** 2
        +
        (path_y - robot_y) ** 2
    )

    nearest_index = np.argmin(distances)

    return (
        nearest_index,
        distances[nearest_index]
    )


def calculate_rmse(error):
    """
    Calculate root mean square error.
    """

    if len(error) == 0:
        return 0.0

    return np.sqrt(
        np.mean(error ** 2)
    )


class VehicleController:

    def __init__(
        self,
        controller_params,
        dt=0.01,
        simulation_time=32.0,
        velocity=0.75,
        wheelbase=0.8
    ):

        self.dt = dt
        self.simulation_time = simulation_time
        self.velocity = velocity
        self.wheelbase = wheelbase

        # Controller parameters
        self.kp = controller_params["kp"]
        self.ki = controller_params["ki"]
        self.kd = controller_params["kd"]

        self.max_steering = (
            controller_params["max_steering"]
        )

        self.max_steering_rate = (
            controller_params["max_steering_rate"]
        )

        self.integral_limit = (
            controller_params["integral_limit"]
        )

        # PID controller
        self.pid = PIDController(
            self.kp,
            self.ki,
            self.kd,
            self.integral_limit
        )

    def simulate(self, track_type):
        """
        Run the vehicle simulation.

        Returns
        -------
        dict
            Simulation results.
        """

        # ------------------------------------------------
        # CREATE PATH
        # ------------------------------------------------

        path_x, path_y = (
            create_reference_path(
                track_type
            )
        )

        # ------------------------------------------------
        # INITIAL POSITION
        # ------------------------------------------------

        robot_x = path_x[0]
        robot_y = path_y[0]

        path_dx = (
            path_x[1]
            - path_x[0]
        )

        path_dy = (
            path_y[1]
            - path_y[0]
        )

        robot_theta = np.arctan2(
            path_dy,
            path_dx
        )

        # Reset PID
        self.pid.reset()

        # ------------------------------------------------
        # HISTORY
        # ------------------------------------------------

        history_x = []
        history_y = []
        history_theta = []
        history_error = []
        history_time = []
        history_steering = []

        # ------------------------------------------------
        # SIMULATION VARIABLES
        # ------------------------------------------------

        num_steps = int(
            self.simulation_time / self.dt
        )

        previous_steering = 0.0

        # ------------------------------------------------
        # SIMULATION LOOP
        # ------------------------------------------------

        for step in range(num_steps):

            current_time = (
                step * self.dt
            )

            # --------------------------------------------
            # FIND NEAREST POINT
            # --------------------------------------------

            nearest_index, distance = (
                find_nearest_point(
                    robot_x,
                    robot_y,
                    path_x,
                    path_y
                )
            )

            # --------------------------------------------
            # VECTOR FROM PATH TO ROBOT
            # --------------------------------------------

            dx = (
                robot_x
                - path_x[nearest_index]
            )

            dy = (
                robot_y
                - path_y[nearest_index]
            )

            # --------------------------------------------
            # PATH DIRECTION
            # --------------------------------------------

            if nearest_index < len(path_x) - 1:

                path_dx = (
                    path_x[nearest_index + 1]
                    -
                    path_x[nearest_index]
                )

                path_dy = (
                    path_y[nearest_index + 1]
                    -
                    path_y[nearest_index]
                )

            else:

                path_dx = (
                    path_x[nearest_index]
                    -
                    path_x[nearest_index - 1]
                )

                path_dy = (
                    path_y[nearest_index]
                    -
                    path_y[nearest_index - 1]
                )

            # --------------------------------------------
            # PATH ANGLE
            # --------------------------------------------

            path_angle = np.arctan2(
                path_dy,
                path_dx
            )

            # --------------------------------------------
            # SIGNED CROSS-TRACK ERROR
            # --------------------------------------------

            cross_track_error = (
                -np.sin(path_angle) * dx
                +
                np.cos(path_angle) * dy
            )

            # --------------------------------------------
            # PID
            # --------------------------------------------

            steering_command = -self.pid.update(
                cross_track_error,
                self.dt
            )

            # --------------------------------------------
            # STEERING ANGLE LIMIT
            # --------------------------------------------

            desired_steering = np.clip(
                steering_command,
                -self.max_steering,
                self.max_steering
            )

            # --------------------------------------------
            # STEERING RATE LIMIT
            # --------------------------------------------

            max_steering_change = (
                self.max_steering_rate
                * self.dt
            )

            steering_change = (
                desired_steering
                - previous_steering
            )

            steering_change = np.clip(
                steering_change,
                -max_steering_change,
                max_steering_change
            )

            steering_angle = (
                previous_steering
                + steering_change
            )

            previous_steering = steering_angle

            # --------------------------------------------
            # BICYCLE MODEL
            # --------------------------------------------

            robot_x += (
                self.velocity
                * np.cos(robot_theta)
                * self.dt
            )

            robot_y += (
                self.velocity
                * np.sin(robot_theta)
                * self.dt
            )

            robot_theta += (
                self.velocity
                / self.wheelbase
                * np.tan(steering_angle)
                * self.dt
            )

            # --------------------------------------------
            # NORMALIZE HEADING
            # --------------------------------------------

            robot_theta = (
                (robot_theta + np.pi)
                % (2 * np.pi)
                - np.pi
            )

            # --------------------------------------------
            # CHECK IF CAR LOST TRACK
            # --------------------------------------------

            if distance > 3.0:

                print(
                    "\nCar lost the track."
                )

                break

            # --------------------------------------------
            # SAVE HISTORY
            # --------------------------------------------

            history_x.append(robot_x)
            history_y.append(robot_y)
            history_theta.append(robot_theta)
            history_error.append(cross_track_error)
            history_time.append(current_time)
            history_steering.append(steering_angle)

        # ------------------------------------------------
        # RMSE
        # ------------------------------------------------

        error_array = np.array(
            history_error
        )

        rmse = calculate_rmse(
            error_array
        )

        # ------------------------------------------------
        # RESULTS
        # ------------------------------------------------

        return {

            "path_x": path_x,

            "path_y": path_y,

            "robot_x": np.array(
                history_x
            ),

            "robot_y": np.array(
                history_y
            ),

            "robot_theta": np.array(
                history_theta
            ),

            "error": error_array,

            "time": np.array(
                history_time
            ),

            "steering": np.array(
                history_steering
            ),

            "rmse": rmse,

            "controller_params": {
                "kp": self.kp,
                "ki": self.ki,
                "kd": self.kd,
                "max_steering": self.max_steering,
                "max_steering_rate":
                    self.max_steering_rate,
                "integral_limit":
                    self.integral_limit
            }
        }