from dataclasses import dataclass
from typing import dataclass_transform
import numpy as np

@dataclass
class State:

    """
    Represents state of bike

    ...

    Attributes:
    ___________
    position : list
        Two element list containing the position coordinates of the bike [x, y] (m)
    velocity: list
        Two element list containing the velocity components of the bike [x_vel, y_vel] (m/s)
    angle : number
        Angle of the front wheel relative to the frame (deg)
    """

    def __init__(self, position, velocity, angle):
        self.position = np.array(position[0], position[1])
        self.velocity = np.array(velocity[0], velocity[1])
        self.angle = angle

    def speed(self):
        return np.linalg.norm(self.velocity)

@dataclass
class Differential

    """
    Represents a differential (difference between two states)

    ...
    Attributes:
    position : list
        Two element list containing the difference in position coordinates (m)
    velocity: list
        Two element list containing the difference in velocity components (m/s)
    angle : number
        Difference in angle between states (deg)
    """

    # Construct a new Differential object
    def __init__(self, position_diff, velocity_diff, angle_diff):
        self.position_diff = np.array(position_diff[0], position_diff[1])
        self.velocity_diff = np.array(velocity_diff[0], velocity_diff[1])
        self.angle_diff = angle_diff


def get_playable_actions(current_state, differentials, timestep):

    """
    Get list of reachable states.

    Extended description...

    Attributes:
    current_state (State)           : State object representing the current state
    differentials (Differential)    : Differential object representing difference between states
    timestep (number)               : Timestep between computations

    Returns:
    reachable_states (list of State)    : list of reachable states
    """

    # Constants
    SPEED_MIN           = 3  # Miniumum possible speed required to keep bike moving (m/s)
    SPEED_MAX           = 10 # Maximum possible speed of the bike (m/s). Assume speed is always less than this

    ACCELERATION_MAX    = 1  # Maximum acceleration of the bike (m/s/s)
    BRAKING_MAX         = 1  # Maximum deacceleration with brakes (m/s/s)

    TURN_ANGLE_MAX  = 60 # Maximum turn angle left and right of center +- 60 (deg)
    TURN_ACC_MAX = max(1, 20 - current_state.speed() * 0.3) # Maximum acceleration of bike's front wheel (def/s)
    # In theory? the turning acceleration depends on th current velocity of the bike
    # Currently some bodged example - not based on physics

    # Calculate bounds
    speed_min = max(SPEED_MIN, current_state.speed() * ACCELERATION_MAX * timestep) # Minimum reachable speed
    speed_max = min(SPEED_MAX, current_state.speed() * BRAKING_MAX * timestep) # Maximum reachable velocity

    clock_angle_max = min(TURN_ANGLE_MAX, current_state.angle + TURN_ACC_MAX * timestep)
    counter_angle_max = max(-TURN_ANGLE_MAX, current_state.angle - TURN_ACC_MAX * timestep)
