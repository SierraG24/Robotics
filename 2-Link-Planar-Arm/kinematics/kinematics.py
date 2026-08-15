import numpy as np

from config import (
    L1,
    L2,
    THETA1_MIN,
    THETA1_MAX,
    THETA2_MIN,
    THETA2_MAX
)

# Angle convention:
#
# theta1:
#   Angle of Link 1 relative to +x axis.
#
# theta2:
#   Angle of Link 2 relative to Link 1.
#
# Therefore:
#   absolute Link 2 angle = theta1 + theta2
#
# Positive angles rotate counterclockwise.



def forward_kinematics(theta1, theta2):
    # Check to see if paramaters are valid
    if not (THETA1_MIN < theta1 < THETA1_MAX):
        raise ValueError("Theta 1 is outside its limits")
    if not (THETA2_MIN < theta2 < THETA2_MAX):
        raise ValueError("Theta 2 is outside its limits")
    
    # Determine the x-coordinate of the elbow
    x_elbow = L1 * np.cos(theta1)

    # Determine the y-coordinate of the elbow.
    y_elbow = L1 * np.sin(theta1)

    # Determine the absolute orientation of Link 2.
    theta2_absolute = theta1 + theta2
    x_l2 = L2 * np.cos(theta2_absolute)
    y_l2 = L2 * np.sin(theta2_absolute)

    # Determine the x-coordinate and y-coordinate of the end effector.
    x_end = x_elbow + x_l2
    y_end = y_elbow + y_l2

    # Return the end-effector x and y coordinates.
    return x_end, y_end

def inverse_kinematics(x, y, current_theta1 = 0.0, current_theta2 = 0.0):
    # Calculate distance from base to target
    total_dist = np.sqrt(x ** 2 + y ** 2)

    if total_dist > L1 + L2:
        raise ValueError("Point is out of reach of arm")

    if total_dist < abs(L1 - L2):
        raise ValueError("Point is too close to the base")

    # Calculate theta 2
    # The numerator and denominator calculate cos alpha
    # cos(theta2) = -cos(alpha)
    # theta2 = arccos(-cos(alpha))
    numerator = (total_dist ** 2 - L1 ** 2 - L2 ** 2)
    denom = 2 * L1 * L2
    cos_alpha = numerator / denom
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)

    # Two possible solutions due to cosine being symmetical
    theta2_1 = np.arccos(cos_alpha)
    theta2_2 = -np.arccos(cos_alpha)

    gamma = np.arctan2(y, x)
    solutions = []

    # Solution 1
    beta = np.arctan2(L2 * np.sin(theta2_1), L1 + L2 * np.cos(theta2_1))
    theta1_1 = gamma - beta

    if (
        THETA1_MIN < theta1_1 < THETA1_MAX
        and
        THETA2_MIN < theta2_1 < THETA2_MAX
    ):
        solutions.append((theta1_1, theta2_1))

    # Solution 2
    beta = np.arctan2(L2 * np.sin(theta2_2), L1 + L2 * np.cos(theta2_2))
    theta1_2 = gamma - beta

    if (
        THETA1_MIN < theta1_2 < THETA1_MAX
        and
        THETA2_MIN < theta2_2 < THETA2_MAX
    ):
        solutions.append((theta1_2, theta2_2))

    # No valid solution check 
    if len(solutions) == 0:
        return None

    # Choose a solution
    def joint_distance(solution):

        theta1, theta2 = solution

        theta1_difference = (
            theta1 - current_theta1
        )

        theta2_difference = (
            theta2 - current_theta2
        )

        return (
            theta1_difference**2
            + theta2_difference**2
        )

    # Select solution requiring the least
    # amount of joint movement
    best_solution = min(
        solutions,
        key=joint_distance)

    return best_solution