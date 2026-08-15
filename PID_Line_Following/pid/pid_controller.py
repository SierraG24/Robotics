import numpy as np


class PIDController:

    def __init__(
        self,
        kp,
        ki,
        kd,
        integral_limit
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral_limit = integral_limit

        self.integral_error = 0.0
        self.previous_error = 0.0

    def reset(self):
        """Reset the internal PID state."""

        self.integral_error = 0.0
        self.previous_error = 0.0

    def update(self, error, dt):
        """
        Calculate PID control output.

        Parameters
        ----------
        error : float
            Current control error.
        dt : float
            Time step in seconds.

        Returns
        -------
        float
            PID control output.
        """

        # Proportional
        proportional = self.kp * error

        # Integral
        self.integral_error += error * dt

        # Prevent integral windup
        self.integral_error = np.clip(
            self.integral_error,
            -self.integral_limit,
            self.integral_limit
        )

        integral = self.ki * self.integral_error

        # Derivative
        if dt > 0:
            derivative_error = (
                error - self.previous_error
            ) / dt
        else:
            derivative_error = 0.0

        derivative = self.kd * derivative_error

        # PID output
        output = (
            proportional
            + integral
            + derivative
        )

        self.previous_error = error

        return output