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
    if not (THETA1_MIN <= theta1 <= THETA1_MAX):
        raise ValueError("Theta 1 is outside its limits")
    if not (THETA2_MIN <= theta2 <= THETA2_MAX):
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

# You can pick between elbow up or down down to due cosine being symmetrical 

def elbow_up_IK(x, y):
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
    numerator = (total_dist**2 - L1**2 - L2**2)
    denom = 2 * L1 * L2
    cos_alpha = numerator / denom
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    theta2 = -np.arccos(cos_alpha)

    # Calculate theta1, which is dependent on theta2
    # Calculate angle from base to target, gamma
    # beta is the angle formed from side l2 * sin(theta2) and L1 + L2 * cos(theta2)
    # theta1 = gamma + beta
    gamma = np.arctan2(y, x)
    numerator = L2 * np.sin(theta2)
    denom = L1 + L2 * np.cos(theta2)
    beta = np.arctan2(numerator, denom)
    theta1 = beta + gamma

    # Check whether theta is within:
    # THETA_MIN <= theta <= THETA_MAX
    if not (THETA1_MIN <= theta1 <= THETA1_MAX):
        return None
    if not (THETA2_MIN <= theta2 <= THETA2_MAX):
        return None

    return theta1, theta2

def elbow_down_IK(x, y):
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
    theta2 = np.arccos(cos_alpha)

    # Calculate theta1, which is dependent on theta2
    # Calculate angle from base to target, gamma
    # beta is the angle formed from side l2 * sin(theta2) and L1 + L2 * cos(theta2)
    # theta1 = gamma - beta
    gamma = np.arctan2(y, x)
    numerator = L2 * np.sin(theta2)
    denom = L1 + L2 * np.cos(theta2)
    beta = np.arctan2(numerator, denom)
    theta1 = gamma - beta

    # Check whether theta is within:
    # THETA_MIN <= theta <= THETA_MAX
    if not (THETA1_MIN <= theta1 <= THETA1_MAX):
        return None
    if not (THETA2_MIN <= theta2 <= THETA2_MAX):
        return None

    return theta1, theta2

def inverse_kinematics(x, y, elbow="up"):

    if elbow == "up":
        return elbow_up_IK(x, y)

    elif elbow == "down":
        return elbow_down_IK(x, y)

    else:
        raise ValueError(
            "Elbow must be 'up' or 'down'"
        )