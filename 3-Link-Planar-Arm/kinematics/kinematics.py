import numpy as np

class Kinematics:
    """
    Kinematics for a 3-link planar robotic arm.

    Joint angles are defined as relative angles:
        theta1 = angle of link 1 from +x axis
        theta2 = angle of link 2 relative to link 1
        theta3 = angle of link 3 relative to link 2

    Therefore:
        link 1 absolute angle = theta1
        link 2 absolute angle = theta1 + theta2
        link 3 absolute angle = theta1 + theta2 + theta3
    """

    def __init__(
        self,
        L1,
        L2,
        L3,
        PHI,
        theta1_min,
        theta_min,
        theta_max
    ):
        """Initialize the kinematics model."""

        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

        self.PHI = PHI

        # Joint limits
        self.THETA1_MIN = theta1_min
        self.THETA_MIN = theta_min
        self.THETA_MAX = theta_max

        # Current joint configuration
        self.current_theta1 = 0.0
        self.current_theta2 = 0.0
        self.current_theta3 = 0.0

    # Current joint configuration
    def setCurrentAngles(self, theta1, theta2, theta3):
        """Set the robot's current joint configuration."""

        self._checkJointLimits(theta1, theta2, theta3)

        self.current_theta1 = theta1
        self.current_theta2 = theta2
        self.current_theta3 = theta3

    def getCurrentAngles(self):
        """Return the robot's current joint configuration."""

        return (
            self.current_theta1,
            self.current_theta2,
            self.current_theta3
        )

    # Check joint limits
    def _checkJointLimits(self, theta1, theta2, theta3):
        """Check whether all joint angles are within their limits."""

        if not (
            self.THETA1_MIN <= theta1 <= self.THETA_MAX
        ):
            raise ValueError(
                "Theta1 value out of range"
            )

        if not (
            self.THETA_MIN <= theta2 <= self.THETA_MAX
        ):
            raise ValueError(
                "Theta2 value out of range"
            )

        if not (
            self.THETA_MIN <= theta3 <= self.THETA_MAX
        ):
            raise ValueError(
                "Theta3 value out of range"
            )

    def getJointPositions(
        self,
        theta1,
        theta2,
        theta3
    ):
        """
        Return the coordinates of the base,
        joints, and end effector.

        Returns
        -------
        positions : list
            [
                (x0, y0),
                (x1, y1),
                (x2, y2),
                (x3, y3)
            ]
        """
        
        self._checkJointLimits(
            theta1,
            theta2,
            theta3
        )

        theta2_absolute = theta1 + theta2
        theta3_absolute = (
            theta1 + theta2 + theta3
        )

        # Base
        x0 = 0.0
        y0 = 0.0

        # Joint 1
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)

        # Joint 2
        x2 = x1 + self.L2 * np.cos(
            theta2_absolute
        )
        y2 = y1 + self.L2 * np.sin(
            theta2_absolute
        )

        # End effector
        x3 = x2 + self.L3 * np.cos(
            theta3_absolute
        )
        y3 = y2 + self.L3 * np.sin(
            theta3_absolute
        )

        return [
            (x0, y0),
            (x1, y1),
            (x2, y2),
            (x3, y3)
        ]
    
    def forwardKinematics(self, theta1, theta2, theta3):
        """
        Calculate the end-effector position.

        Parameters
        ----------
        theta1 : float
            First joint angle.
        theta2 : float
            Second joint angle relative to link 1.
        theta3 : float
            Third joint angle relative to link 2.

        Returns
        -------
        x, y : float
            End-effector position.
        """

        # Check joint limits
        try:
            self._checkJointLimits(
                theta1,
                theta2,
                theta3
            )
        except ValueError as e:
            raise ValueError(f"Invalid joint angles: {e}")

        # Absolute link angles
        theta2_absolute = theta1 + theta2
        theta3_absolute = theta2_absolute + theta3

        # Link 1
        x_l1 = self.L1 * np.cos(theta1)
        y_l1 = self.L1 * np.sin(theta1)

        # Link 2
        x_l2 = self.L2 * np.cos(theta2_absolute)
        y_l2 = self.L2 * np.sin(theta2_absolute)

        # Link 3
        x_l3 = self.L3 * np.cos(theta3_absolute)
        y_l3 = self.L3 * np.sin(theta3_absolute)

        # End effector
        x_final = x_l1 + x_l2 + x_l3
        y_final = y_l1 + y_l2 + y_l3

        return x_final, y_final


    def phiLockedIK(self, x, y):
        """
        Calculate inverse kinematics for a desired x/y position
        while keeping the end-effector orientation fixed at PHI.

        Both elbow-up and elbow-down configurations are calculated.

        Solutions outside the joint limits are removed.

        Of the remaining solutions, the solution closest to the
        robot's current joint configuration is returned.

        Returns
        -------
        theta1, theta2, theta3 : tuple
            Closest valid IK solution.
        """
        # Find wrist position
        x_wrist = x - self.L3 * np.cos(self.PHI)
        y_wrist = y - self.L3 * np.sin(self.PHI)

        r = np.hypot(x_wrist, y_wrist)


        # Check if wrist position is reacheable
        if r > self.L1 + self.L2:
            raise ValueError("Wrist position is outside workspace")
        
        if r < (abs(self.L1- self.L2)):
            raise ValueError("Wrist position is inside unreachable workspace")

        # Now do the same steps as the 2-Link Planar inverse kinematics
        # First find beta, the angle between L1 & L2
        num = (self.L1 ** 2 + self.L2 ** 2 - r ** 2)
        denom = 2 * self.L1 * self.L2
        beta_cos = num / denom
        beta_cos = np.clip(beta_cos, -1.0, 1.0)
        beta = np.arccos(beta_cos)

        # Second sigma, the acute angle from L1 & hypotenuse of L1 & L2
        num = (r ** 2 + self.L1 **2 - self.L2 ** 2)
        denom = 2 * self.L1 * r
        sigma_cos = num / denom
        sigma_cos = np.clip(sigma_cos, -1.0, 1.0)
        sigma = np.arccos(sigma_cos)

        # Third alpha, the angle formed from the right triangle of the base and 
        # the wrist position
        alpha = np.arctan2(y_wrist, x_wrist)

        # Calculate both elbow-up and elbow-down configurations
        # Elbow-down
        theta1_down = alpha - sigma
        theta2_down = np.pi - beta

        # Elbow-up
        theta1_up = alpha + sigma
        theta2_up = beta - np.pi

        # Calculate theta3 for both configurations
        theta3_down = self.PHI - (theta1_down + theta2_down)
        theta3_up = self.PHI - (theta1_up + theta2_up)

        # Check if the configurations are valid
        valid_solutions = []
        solutions = [
            (theta1_down, theta2_down, theta3_down),
            (theta1_up, theta2_up, theta3_up)
        ]
        for theta1, theta2, theta3 in solutions:
            try:
                self._checkJointLimits(theta1, theta2, theta3)
                valid_solutions.append((theta1, theta2, theta3))
            except ValueError:
                continue

        if len(valid_solutions) == 0:
            raise ValueError(
                "No IK solution satisfies the joint limits"
            )

        # Choose the solution closest to the current joint configuration
        current = np.array([self.current_theta1, self.current_theta2, self.current_theta3])

        def distance(solution):
            solution = np.array(solution)

            angle_difference = solution - current

            # Wrap angle differences to [-pi, pi]
            angle_difference = (
                angle_difference + np.pi
            ) % (2 * np.pi) - np.pi

            return np.linalg.norm(angle_difference)

        closest_solution = min(
            valid_solutions,
            key=distance
        )

        return closest_solution    

    def _solveIKForPhi(self, x, y, phi):
        """
        Solve 3-link inverse kinematics for a specific phi.
        Uses the same method as phiLockedIK() to find both elbow-up and elbow-down solutions.
        Unlike phiLockedIK(), this method returns ALL valid
        elbow-up and elbow-down solutions for the given phi.

        Parameters
        ----------
        x, y : float
            Desired end-effector position.

        phi : float
            Desired end-effector orientation in radians.

        Returns
        -------
        valid_solutions : list
            List of valid (theta1, theta2, theta3, phi) solutions.
        """


        # 1. Find wrist position
        x_wrist = x - self.L3 * np.cos(phi)
        y_wrist = y - self.L3 * np.sin(phi)

        r = np.hypot(x_wrist, y_wrist)

        # 2. Check whether wrist is reachable

        if r > self.L1 + self.L2:
            return []

        if r < abs(self.L1 - self.L2):
            return []

        # Avoid division by zero
        if np.isclose(r, 0.0):
            return []

        # 3. Solve 2-link IK
        # beta
        num = (self.L1**2 + self.L2**2 - r**2)
        denom = 2 * self.L1 * self.L2
        beta_cos = num / denom
        beta_cos = np.clip(beta_cos, -1.0, 1.0)
        beta = np.arccos(beta_cos)

        # sigma
        num = (r**2 + self.L1**2 - self.L2**2)
        denom = 2 * self.L1 * r
        sigma_cos = num / denom
        sigma_cos = np.clip(sigma_cos, -1.0, 1.0)
        sigma = np.arccos(sigma_cos)

        # alpha
        alpha = np.arctan2(y_wrist,x_wrist)

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
            (theta1_down, theta2_down, theta3_down, phi),
            (theta1_up, theta2_up,theta3_up,phi)
        ]

        # 7. Check joint limits
        valid_solutions = []

        for theta1, theta2, theta3, phi_value in solutions:

            try:
                self._checkJointLimits(theta1,theta2,theta3)
                valid_solutions.append((theta1, theta2, theta3, phi_value))
            except ValueError:
                continue

        return valid_solutions


    def findAllPhiSolutions(
        self,
        x,
        y,
        phi_min=0.0,
        phi_max=360.0,
        phi_step=1.0
    ):
        """
        Search through possible phi values and return
        every valid IK configuration.

        Parameters
        ----------
        x, y : float
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
                (theta1, theta2, theta3, phi),
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
        x, y : float
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
            (theta1, theta2, theta3, phi)

            All angles are returned in radians.
        """
        # Find every valid configuration

        valid_solutions = self.findAllPhiSolutions(
            x,
            y,
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
            self.current_theta1,
            self.current_theta2,
            self.current_theta3
        ])

        # Calculate distance from current configuration

        def configurationDistance(solution):

            theta1, theta2, theta3, phi = solution

            candidate = np.array([
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
