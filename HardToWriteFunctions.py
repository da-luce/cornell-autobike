from dataclasses import dataclass
import numpy as np
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from datetime import datetime

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


class Model:

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
    DIST_REAR_AXEL = 0.85 # m
    DIST_FRONT_AXEL = 0.85 # m
    WHEEL_BASE = DIST_REAR_AXEL + DIST_FRONT_AXEL

    MASS = 15 # kg

    CORNERING_STIFF_FRONT = 16.0 * 2.0  # N/rad
    CORNERING_STIFF_REAR = 17.0 * 2.0  # N/rad

    YAW_INERTIA = 22 # kg/(m^2)

    # Input constraints
    SPEED_MIN = 2 # m/s
    SPEED_MAX = 10 # m/s

    STEER_ANGLE_MAX = np.radians(37) # rad

    ACCELERATION_MIN = -5 # m/(s^2)
    ACCELERATION_MAX = 5 # m/(s^2)
 
    STEER_ACC_MIN = -5
    STEER_ACC_MAX = 5

    DT = 0.5 # (s)

    # Figure
    fig, ax = plt.subplots()

    @staticmethod
    # Return updated state based off inputs
    def update_state(current_state, throttle, steering):

        x,y            = current_state.x, current_state.y
        vel_x,vel_y    = current_state.vel_x, current_state.vel_y
        yaw_angle      = current_state.yaw_angle
        steering_angle = current_state.steering_angle

        # Advect bike
        x = x + vel_x * math.cos(yaw_angle) * Model.DT - vel_y * math.sin(yaw_angle) * Model.DT
        y = y + vel_x * math.sin(yaw_angle) * Model.DT + vel_y * math.cos(yaw_angle) * Model.DT

        yaw = yaw_angle * steering_angle * Model.DT
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi # Normalize yaw angle

        lat_force_front = -Model.CORNERING_STIFF_FRONT * math.atan2(((vel_y + Model.DIST_FRONT_AXEL * steering_angle) / vel_x - steering), 1.0)
        lat_force_rear = -Model.CORNERING_STIFF_REAR * math.atan2((vel_y - Model.DIST_REAR_AXEL * steering_angle) / vel_x, 1.0)

        # Aerodynamic and friction coefficients
        R_x = 0.01 * vel_x
        F_aero = 1.36 * vel_x ** 2
        F_load = F_aero + R_x

        vel_x = vel_x + (throttle - lat_force_front * math.sin(steering) / Model.MASS - F_load / Model.MASS + vel_y * steering) * Model.DT
        vel_y = vel_y + (lat_force_rear / Model.MASS + lat_force_front * math.cos(steering) / Model.MASS - vel_x * steering) * Model.DT
        steering = steering + (lat_force_front * Model.DIST_FRONT_AXEL * math.cos(steering) - lat_force_rear * Model.DIST_REAR_AXEL) / Model.YAW_INERTIA * Model.DT

        # Advect bike
        x = x + vel_x * math.cos(yaw_angle) * Model.DT - vel_y * math.sin(yaw_angle) * Model.DT
        y = y + vel_x * math.sin(yaw_angle) * Model.DT + vel_y * math.cos(yaw_angle) * Model.DT

        return State(x, y, vel_x, vel_y, yaw, steering)

    @staticmethod
    # Return all possbile states in next time step
    def get_possible_states(current_state, differentials):

        start_time = datetime.now()

        possible_states = []
        # Loop through possible inputs
        for acc in np.arange(Model.ACCELERATION_MIN, Model.ACCELERATION_MAX, 0.1):
            for steer in np.arange(-Model.STEER_ANGLE_MAX, Model.STEER_ANGLE_MAX, 0.1):
                state = Model.update_state(current_state, acc, steer)
                if not Model.validState(state):
                    continue
                possible_states.append(state)
                if True:
                    state.plot()

        end_time = datetime.now()
        print('Got all possible states in {}'.format(end_time - start_time))
        return possible_states

    # Check state invariants
    @staticmethod
    def validState(state):
        return Model.SPEED_MIN <= state.speed() <= Model.SPEED_MAX

    @staticmethod
    def plotBike(state):

        wheel_width = 0.1
        wheel_length = 0.6

        rear_axel_x = state.x - Model.DIST_REAR_AXEL * math.cos(state.yaw_angle)
        rear_axel_y = state.y - Model.DIST_REAR_AXEL * math.sin(state.yaw_angle)

        front_axel_x = state.x + Model.DIST_FRONT_AXEL * math.cos(state.yaw_angle)
        front_axel_y = state.y + Model.DIST_FRONT_AXEL * math.sin(state.yaw_angle)

        # Plot center line
        Model.ax.plot([rear_axel_x, front_axel_x], [rear_axel_y, front_axel_y], marker = 'o')

        rear_wheel = Model.rectangle(rear_axel_x, rear_axel_y, wheel_width, wheel_length, "red", state.yaw_angle)
        front_wheel = Model.rectangle(front_axel_x, front_axel_y, wheel_width, wheel_length, "blue", state.yaw_angle + state.steering_angle) # Why neg steering?

        Model.ax.add_patch(rear_wheel)
        Model.ax.add_patch(front_wheel)

        Model.ax.plot([state.x, state.x+state.vel_x], [state.y, state.y+state.vel_y], marker='>')

        Model.ax.set_xlim(-2,5)
        Model.ax.set_ylim(-2,5)

    @staticmethod
    def rectangle(centerX, centerY, width, length, color, rotation):

        rect = patches.Rectangle((centerX - length/2, centerY - width /2), length, width, color = color, alpha = 0.8)
        ts = Model.ax.transData.transform([centerX, centerY])
        transform = mpl.transforms.Affine2D().rotate_around(centerX, centerY, rotation) + Model.ax.transData
        rect.set_transform(transform)

        return rect


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

    def __init__(self, x, y, vel_x, vel_y, yaw_angle, steering_angle):
        self.x = x
        self.y = y
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.yaw_angle = yaw_angle
        self.steering_angle = steering_angle

        self.slip_angle = VectorUtils.angle_between([x,y], [vel_y, vel_y]) - yaw_angle 

    def speed(self):
        return math.sqrt(self.vel_x * self.vel_x + self.vel_y * self.vel_y)

    def __str__(self):
        return f"""Position: ({self.x}, {self.y})
                Velocity: ({self.vel_x}, {self.vel_y})
                Yaw: {self.yaw_angle}
                Steering: {self.steering_angle}"""

    def plot(self):
        # Model.ax.quiver(self.x, self.y, self.vel_x, self.vel_y, width=0.001, scale=1/(self.speed()*0.01))
        Model.ax.plot(self.x, self.y, 'o', color = "blue", alpha = 0.25)


# TODO: is different resolution of x/y components desired?
@dataclass
class Differential:

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

if __name__ == "__main__":
    fig = plt.figure()
    state = State(0, 0, 1, 0, np.radians(0), np.radians(0))
    Model.plotBike(state)
    differentials = Differential(1,1,1)
    possible_states = Model.get_possible_states(state, differentials)
    plt.show()

