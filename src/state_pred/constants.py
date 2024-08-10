"""This module contains simulation constants/settings
"""

import numpy as np

# PHYSICAL PROPERTIES

DIST_REAR_AXEL = 0.85  # m
DIST_FRONT_AXEL = 0.85  # m
WHEEL_BASE = DIST_REAR_AXEL + DIST_FRONT_AXEL

MASS = 15  # kg

CORNERING_STIFF_FRONT = 16.0 * 2.0  # N/rad
CORNERING_STIFF_REAR = 17.0 * 2.0  # N/rad

YAW_INERTIA = 22  # kg/(m^2)

# INPUT CONSTRAINTS

SPEED_MIN = 1  # m/s
SPEED_MAX = 100  # m/s

STEER_ANGLE_MAX = np.radians(90)  # rad
STEER_ANGLE_DELTA = np.radians(30)  # rad

ACCELERATION_MIN = 0  # m/(s^2)
ACCELERATION_MAX = 10  # m/(s^2)

STEER_ACC_MIN = -50
STEER_ACC_MAX = 50

DT = 0.3  # (s)
