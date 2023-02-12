from dataclasses import dataclass
from typing import dataclass_transform
import numpy as np

class VectorUtils:

    """
    Class containing common vector operations
    """
    @staticmethod
    def unit_vector(vector):
        """
        Returns the unit vector of the vector.
        """
        return vector / np.linalg.norm(vector)

    @staticmethod
    def angle_between(v1, v2):
        """
        Returns the angle in radians between vectors 'v1' and 'v2'
        """
        v1_u = VectorUtils.unit_vector(v1)
        v2_u = VectorUtils.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

class Bicycle:

    """
    Contains bicycle constants for dynamic model

    ...

    Attributes:
    ___________
    DIST_FRONT_AXEL: number
        Positive distance from the center of mass to the rear axel
    DIST_REAR_AXEL: number
        Positive distance from the center of mass to the front axel

    """

    # Physical properties
    DIST_REAR_AXEL = 0.75 # m
    DIST_FRONT_AXEL = 0.50 # m
    WHEEL_BASE = DIST_REAR_AXEL + DIST_FRONT_AXEL

    MASS = 15 # kg

    CORNERING_STIFF_FRONT = 16.0 * 2.0  # N/rad
    CORNERING_STIFF_REAR = 17.0 * 2.0  # N/rad

    YAW_INERTIA = 22 # kg/(m^2)

    # Input constraints
    SPEED_MIN = 2 # m/s
    SPEED_MAX = 5 # m/s

    STEER_ANGLE_MAX = np.radians(37) # rad

    ACCELERATION_MIN = -1.5 # m/(s^2)
    ACCELERATION_MAX = 1 # m/(s^2)

    # Create a new bicycle
    def __init__(self, initial_state)
        pass

    @staticmethod
    # Return updated state based off inputs
    def update_state(throttle, steering):
        pass

    @staticmethod
    # Return all possbile states in next time step
    def get_possible_states():
        pass



@dataclass
class State:

    """
    Represents state of bike

    ...

    Attributes:
    ___________
    position : np.array
        Position vector of the bike's center of mass [x, y] (m)
    velocity: list
        Velocity vector of the bike [x, y] (m/s)
    yaw_angle : number
        Orientation of the bike with respect to x-axis (rad)
        Also called the "heading angle"
    steering_angle : number
        Orientation of the bike's front wheel with respect to th frame (rad)
    """

    def __init__(self, position, velocity, yaw_angle, steering_angle):
        self.position = position
        self.velocity = velocity
        self.yaw_angle = yaw_angle
        self.steering = steering_angle

        self.slip_angle = VectorUtils.angle_between(velocity, velocity) - yaw_angle 

    def speed(self):
        return np.linalg.norm(self.velocity)

# TODO: is different resolution of x/y components desired?
@dataclass
class Differential

    """
    Represents a differential (constant difference between all states in state matrix)
    Assume that position_diff >= velocity_diff
    ...
    Attributes:
    position_diff : number
        Difference of position coordinates between states in matrix (m)
    velocity_diff: number
        Difference of velocity components between states in matrix (m/s)
    angle : number
        Difference in angle between states in matrix (deg)
    """

    # Construct a new Differential object
    def __init__(self, position_diff, velocity_diff, angle_diff):
        self.position_diff = position_diff
        self.velocity_diff = velocity_diff
        self.angle_diff = angle_diff

"""
def get_playable_actions(current_state, differentials, timestep):

    ###
    Get list of reachable states.

    Extended description...

    Assumptions:
        - Constants are constant (e.g. in reality, braking will be impacted by weather conditions)
        - Motors turning front wheel and moving drive train provide instant torque
        - Friction involved turning the front wheel and other instances is ignored
        - It is impossible to accelerate and brake at the same time
        - Bike never exceeds SPEED_MAX and is never slower than SPEED_MIN

    Attributes:
    current_state (State)           : State object representing the current state
    differentials (Differential)    : Differential object representing difference between states
                                      (Essentially the resolution of the state matrix)
    timestep (number)               : Timestep between computations

    Returns:
    reachable_states (list of State)    : list of reachable states
    ###

    # Constants
    SPEED_MIN           = 3  # Miniumum possible speed required to keep bike moving (m/s)
    SPEED_MAX           = 10 # Maximum possible speed of the bike (m/s).

    ACCELERATION_MAX    = 1  # Maximum acceleration of the bike (m/s/s)
    BRAKING_MAX         = 1  # Maximum deacceleration with brakes (m/s/s)

    TURN_ANGLE_MAX  = 60 # Maximum turn angle left and right of center +- 60 (deg)
    TURN_ACC_MAX = max(1, 20 - current_state.speed() * 0.3) # Maximum acceleration of bike's front wheel (def/s)
    # In theory (?) the turning acceleration is limited by the current velocity of the bike and depends on torque of turning motor
    # E.g. we do dont want to turn the front wheel quickly when the bike is moving fast
    # Currently some bodged example - not based on physics (or anything really)

    # Calculate bounds
    speed_min = max(SPEED_MIN, current_state.speed() * ACCELERATION_MAX * timestep) # Minimum reachable speed
    speed_max = min(SPEED_MAX, current_state.speed() * BRAKING_MAX * timestep) # Maximum reachable velocity

    clock_angle_max = min(TURN_ANGLE_MAX, current_state.angle + TURN_ACC_MAX * timestep)
    counter_angle_max = max(-TURN_ANGLE_MAX, current_state.angle - TURN_ACC_MAX * timestep)

    # Calculate resolutions based of differentials
    RES_ACCELERATION = differentials.velocity_diff / 2
    RES_TURNING = differentials.angle_diff / 2

    # Return all possible states reachable in one timestep
    # All states reachable by braking
    # All states reachable by cruising
    # All states reachable by accelerating

    reachable_states = []

    # Vary amount of acc/breaking
    for acc in range(-BRAKING_MAX, ACCELERATION_MAX, RES_ACCELERATION):

        speed = current_state.speed() + 0.5 * acc * timestep * timestep

        # If we cannot go that slow/fast, ignore
        if speed not in range(SPEED_MIN, SPEED_MAX):
            continue

        # Vary how much we turn
        for turn in range(-TURN_ACC_MAX, TURN_ACC_MAX, RES_TURNING):

            angle = current_state.angle + 0.5 * turn * timestep * timestep

            # If we cannot turn that far, ignore
            if angle not in range(-TURN_ANGLE_MAX, TURN_ANGLE_MAX):
                continue

            velocity = current_state.velocity + np.array(
                        (0.5 * acc * timestep * timestep) * math.cos(angle),
                        (0.5 * acc * timestep * timestep) * math.sin(angle)
                    ) 
            reachable_states.append(State([],[],angle))
"""
