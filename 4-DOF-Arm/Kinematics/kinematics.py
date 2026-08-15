import numpy as np

class Kinematics:
    """
    Kinematics for a 3-link planar robotic arm with a revolute joint for the base.

    Joint angles are defined as relative angles:
        theta_base = angle of the base link from +x axis to +y axis.
        theta1 = angle of link 1 from +x axis
        theta2 = angle of link 2 relative to link 1
        theta3 = angle of link 3 relative to link 2

    Therefore:
        link 1 absolute angle = theta1
        link 2 absolute angle = theta2 + theta1
        link 3 absolute angle = theta3 + theta2 + theta1
    """

    # Phi is derived from the best solution of the inverse kinematics for a given (x, y) position.
    def __init__(
        self,
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
    ):
        """Initialize the kinematics model."""

        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

        self.__phi = 0.0  # Default end-effector orientation in radians

        # Joint limits
        self.theta_base_min = THETA_BASE_MIN
        self.theta_base_max = THETA_BASE_MAX
        self.theta1_min = THETA1_MIN
        self.theta2_min = THETA2_MIN
        self.theta3_min = THETA3_MIN
        self.theta1_max = THETA1_MAX
        self.theta2_max = THETA2_MAX
        self.theta3_max = THETA3_MAX    

        # Current joint configuration
        # Start horizontally with the arm fully extended on the x-axis
        self.current_theta_base = 0.0
        self.current_theta1 = 0.0
        self.current_theta2 = 0.0
        self.current_theta3 = 0.0

    # Current joint configuration
    def setCurrentAngles(self, theta_base, theta1, theta2, theta3):
        """Set the robot's current joint configuration."""

        self._checkJointLimits(theta_base, theta1, theta2, theta3)

        self.current_theta_base = theta_base
        self.current_theta1 = theta1
        self.current_theta2 = theta2
        self.current_theta3 = theta3

    def getCurrentAngles(self):
        """Return the robot's current joint configuration."""

        return (
            self.current_theta_base,
            self.current_theta1,
            self.current_theta2,
            self.current_theta3
        )

    # Check joint limits
    def _checkJointLimits(self, theta_base, theta1, theta2, theta3):
        """Check whether all joint angles are within their limits."""
        if not (self.theta_base_min <= theta_base <= self.theta_base_max):
            raise ValueError(
                f"theta_base={theta_base} is out of limits "
                f"[{self.theta_base_min}, {self.theta_base_max}]"
            )
        if not (self.theta1_min <= theta1 <= self.theta1_max):
            raise ValueError(
                f"theta1={theta1} is out of limits "
                f"[{self.theta1_min}, {self.theta1_max}]"
            )

        if not (self.theta2_min <= theta2 <= self.theta2_max):
            raise ValueError(
                f"theta2={theta2} is out of limits "
                f"[{self.theta2_min}, {self.theta2_max}]"
            )

        if not (self.theta3_min <= theta3 <= self.theta3_max):
            raise ValueError(
                f"theta3={theta3} is out of limits "
                f"[{self.theta3_min}, {self.theta3_max}]"
            )

    # Same as forward kinematics, but returns the positions of all joints and the end effector
    def getJointPositions(
        self,
        theta_base,
        theta1,
        theta2,
        theta3
    ):
        """
        Return the 3D coordinates of the base,
        joints, and end effector.

        Parameters
        ----------
        theta_base : float
            Base rotation angle around the z-axis.

        theta1 : float
            Shoulder joint angle.

        theta2 : float
            Elbow joint angle relative to link 1.

        theta3 : float
            Wrist joint angle relative to link 2.

        Returns
        -------
        positions : list
            [
                (x0, y0, z0),
                (x1, y1, z1),
                (x2, y2, z2),
                (x3, y3, z3)
            ]
        """

        self._checkJointLimits(
            theta_base,
            theta1,
            theta2,
            theta3
        )

        # Absolute angles within the planar arm
        theta2_absolute = theta1 + theta2

        theta3_absolute = (
            theta1 +
            theta2 +
            theta3
        )

         # Base
        x0 = 0.0
        y0 = 0.0
        z0 = 0.0

        # Link 1
        r1 = self.L1 * np.cos(theta1)
        z1 = self.L1 * np.sin(theta1)

        x1 = r1 * np.cos(theta_base)
        y1 = r1 * np.sin(theta_base)

        # Link 2
        r2 = (r1 + self.L2 * np.cos(theta2_absolute))
        z2 = ( z1 + self.L2 * np.sin(theta2_absolute))

        x2 = r2 * np.cos(theta_base)
        y2 = r2 * np.sin(theta_base)

        # Link 3 / End Effector
        r3 = (r2 + self.L3 * np.cos(theta3_absolute))
        z3 = (z2 + self.L3 * np.sin(theta3_absolute))

        x3 = r3 * np.cos(theta_base)
        y3 = r3 * np.sin(theta_base)

        return [
            (x0, y0, z0),
            (x1, y1, z1),
            (x2, y2, z2),
            (x3, y3, z3)
        ]
    
    def _solveIKForPhi(self, x, y, z, phi):
        """
        Solve 3-link and rotating base inverse kinematics for a specific phi.
        This method returns ALL valid elbow-up and elbow-down solutions for
        the given phi.

        Parameters
        ----------
        x, y, z: float
            Desired end-effector position.

        phi : float
            Desired end-effector orientation in radians.

        Returns
        -------
        valid_solutions : list
            List of valid (theta1, theta2, theta3, phi) solutions.
        """

        # 1. Find theta base
        theta_base = np.arctan2(y, x)

        # Use the same formula as the inverse kinematics for a 2-link planar arm to find theta1 and theta2
        # Instead of x and y, we use the projection of the end effector position onto the plane of the arm 
        # (r', z') where r' = sqrt(x^2 + y^2) and z' = z
        r = np.hypot(x, y)  # Distance from the base to the end effector in the xy-plane

        # 2. Find wrist position
        r_wrist =  r - self.L3 * np.cos(phi)
        z_wrist =  z - self.L3 * np.sin(phi)

        # 2. Check whether wrist is reachable

        wrist_distance = np.hypot(r_wrist, z_wrist)
        if wrist_distance > self.L1 + self.L2:
            return []

        if wrist_distance < abs(self.L1 - self.L2):
            return []

        if np.isclose(wrist_distance, 0.0):
            return []

        # 3. Solve 2-link IK
        # beta
        num = (self.L1**2 + self.L2**2 - wrist_distance**2)
        denom = 2 * self.L1 * self.L2
        beta_cos = num / denom
        beta_cos = np.clip(beta_cos, -1.0, 1.0)
        beta = np.arccos(beta_cos)

        # sigma
        num = (wrist_distance**2 + self.L1**2 - self.L2**2)
        denom = 2 * self.L1 * wrist_distance
        sigma_cos = num / denom
        sigma_cos = np.clip(sigma_cos, -1.0, 1.0)
        sigma = np.arccos(sigma_cos)

        # alpha
        alpha = np.arctan2(z_wrist, r_wrist)

        # 4. Calculate elbow-down solution

        theta1_down = alpha - sigma
        theta2_down = np.pi - beta

        # 5. Calculate elbow-up solution
        theta1_up = alpha + sigma
        theta2_up = beta - np.pi

        # 6. Calculate theta3
        theta3_down = (phi - theta1_down - theta2_down)
        theta3_up = (phi- theta1_up - theta2_up)

        solutions = [
            (theta_base, theta1_down, theta2_down, theta3_down, phi),
            (theta_base, theta1_up, theta2_up, theta3_up, phi)
        ]

        # 7. Check joint limits
        valid_solutions = []

        for theta_base, theta1, theta2, theta3, phi_value in solutions:

            try:
                self._checkJointLimits(theta_base, theta1, theta2, theta3)
                valid_solutions.append((theta_base, theta1, theta2, theta3, phi_value))
            except ValueError:
                continue

        return valid_solutions


    def findAllPhiSolutions(
        self,
        x,
        y,
        z,
        phi_min=0.0,
        phi_max=360.0,
        phi_step=1.0
    ):
        """
        Search through possible phi values and return
        every valid IK configuration.

        Parameters
        ----------
        x, y, z : float
            Desired end-effector position.

        phi_min : float
            Minimum phi in degrees.

        phi_max : float
            Maximum phi in degrees.

        phi_step : float
            Increment between phi values in degrees.

        Returns
        -------
        valid_solutions : list
            List of tuples:

            [
                (theta_base, theta1, theta2, theta3, phi),
                ...
            ]

            All angles are returned in radians.
        """

        if phi_step <= 0:
            raise ValueError(
                "phi_step must be greater than zero"
            )

        if phi_min > phi_max:
            raise ValueError(
                "phi_min must be less than phi_max"
            )

        valid_solutions = []

        # Generate phi values in degrees
        phi_values = np.arange(
            phi_min,
            phi_max + phi_step,
            phi_step
        )

        for phi_degrees in phi_values:

            # Convert degrees to radians
            phi = np.deg2rad(phi_degrees)

            # Solve IK for this phi value
            solutions = self._solveIKForPhi(
                x,
                y,
                z,
                phi
            )

            # Add valid solutions to the list
            valid_solutions.extend(
                solutions
            )

        return valid_solutions


    def findBestPhiValueIK(
        self,
        x,
        y,
        z,
        phi_min=0.0,
        phi_max=360.0,
        phi_step=1.0
    ):
        """
        Search through possible phi values and choose
        the valid configuration closest to the current
        joint configuration.

        Parameters
        ----------
        x, y, z : float
            Desired end-effector position.

        phi_min : float
            Minimum phi in degrees.

        phi_max : float
            Maximum phi in degrees.

        phi_step : float
            Increment between phi values in degrees.

        Returns
        -------
        best_solution : tuple
            (theta_base, theta1, theta2, theta3, phi)

            All angles are returned in radians.
        """
        # Find every valid configuration

        valid_solutions = self.findAllPhiSolutions(
            x,
            y,
            z,
            phi_min,
            phi_max,
            phi_step
        )

        if len(valid_solutions) == 0:
            raise ValueError(
                "No IK solution found for any phi value"
            )
        
        # Current configuration

        current = np.array([
            self.current_theta_base,
            self.current_theta1,
            self.current_theta2,
            self.current_theta3
        ])

        # Calculate distance from current configuration

        def configurationDistance(solution):

            theta_base, theta1, theta2, theta3, phi = solution

            candidate = np.array([
                theta_base,
                theta1,
                theta2,
                theta3
            ])

            # See which solution is closest to the current configuration
            difference = candidate - current

            # Wrap angle differences to [-pi, pi]
            difference = (
                difference + np.pi
            ) % (2 * np.pi) - np.pi

            # Return the Euclidean distance
            return np.linalg.norm(difference)

        # Choose closest solution
        best_solution = min(
            valid_solutions,
            key=configurationDistance
        )

        return best_solution
