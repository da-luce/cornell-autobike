from dataclasses import dataclass
import numpy as np
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from datetime import datetime
from scipy.stats import gaussian_kde
from numba import jit

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
    STEER_ANGLE_DELTA = np.radians(5) # rad

    ACCELERATION_MIN = -5 # m/(s^2)
    ACCELERATION_MAX = 5 # m/(s^2)
 
    STEER_ACC_MIN = -5
    STEER_ACC_MAX = 5

    DT = 0.5 # (s)


    # Figure
    fig, ax = plt.subplots()

    dark = '#0B0B0B'
    light = '#C9C9C9'

    red = '#ff3333'
    purple = '#9933ff'
    orange = '#ff9933'
    blue = '#33ccff'

    ax.set_facecolor(dark)
    fig.patch.set_facecolor(dark)

    ax.spines['bottom'].set_color(light)
    ax.spines['top'].set_color(light)
    ax.spines['left'].set_color(light)
    ax.spines['right'].set_color(light)
    ax.xaxis.label.set_color(light)
    ax.yaxis.label.set_color(light)
    ax.tick_params(axis='x', colors=light)
    ax.tick_params(axis='y', colors=light)
    ax.grid(linestyle='dashed')

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


    # Return all possbile states in next time step
    @staticmethod
    def get_possible_states(current_state, differentials, graph):

        start_time = datetime.now()

        possible_acc = np.arange(Model.ACCELERATION_MIN, Model.ACCELERATION_MAX, 0.2)
        possible_steer = np.arange(-Model.STEER_ANGLE_DELTA, Model.STEER_ANGLE_DELTA, np.radians(0.5)) 

        iterable = (Model.update_state(current_state, acc, steer) for acc in possible_acc for steer in possible_steer)
        states = np.fromiter(iterable, State)

        valid_iter = (s for s in states if Model.validState(s))
        possible_states = np.fromiter(valid_iter, State)

        if graph:

            if len(possible_states) != 0:
                x = list(o.x for o in possible_states)
                y = list(o.y for o in possible_states)

                xy = np.vstack([x,y])
                z = gaussian_kde(xy)(xy)

                Model.ax.scatter(x, y, c=z, s=100, alpha=1)

            invalid_iter = (s for s in states if not Model.validState(s)) 
            invalid_states = np.fromiter(invalid_iter, State)
            x_inv = list(o.x for o in invalid_states)
            y_inv = list(o.y for o in invalid_states)


            Model.ax.scatter(x_inv, y_inv, c=Model.red, s=100, alpha=0.05)

            Model.plotBike(current_state)

        end_time = datetime.now()
        print("Got all " + str(len(possible_states)) + " possible states in {}".format(end_time - start_time))

        return possible_states


    # Check state invariants
    @staticmethod
    def validState(state):
        return (Model.SPEED_MIN <= state.speed() <= Model.SPEED_MAX and 
                -Model.STEER_ANGLE_MAX <= state.steering_angle <= Model.STEER_ANGLE_MAX)


    # Plot bike in certain state
    @staticmethod
    def plotBike(state):

        wheel_width = 0.1
        wheel_length = 0.6

        rear_axel_x = state.x - Model.DIST_REAR_AXEL * math.cos(state.yaw_angle)
        rear_axel_y = state.y - Model.DIST_REAR_AXEL * math.sin(state.yaw_angle)

        front_axel_x = state.x + Model.DIST_FRONT_AXEL * math.cos(state.yaw_angle)
        front_axel_y = state.y + Model.DIST_FRONT_AXEL * math.sin(state.yaw_angle)

        # Plot center line
        Model.ax.plot([rear_axel_x, front_axel_x], [rear_axel_y, front_axel_y], marker = 'o', color="white")

        rear_wheel = Model.drawWheel(rear_axel_x, rear_axel_y, wheel_width, wheel_length, Model.red, state.yaw_angle)
        front_wheel = Model.drawWheel(front_axel_x, front_axel_y, wheel_width, wheel_length, Model.blue, state.yaw_angle + state.steering_angle) # Why neg steering?

        Model.ax.add_patch(rear_wheel)
        Model.ax.add_patch(front_wheel)

        Model.ax.plot([state.x, state.x+state.vel_x], [state.y, state.y+state.vel_y], marker='>', color = Model.orange)

        Model.ax.set_xlim(-2,5)
        Model.ax.set_ylim(-2,5)

    # Draw a wheel
    @staticmethod
    def drawWheel(centerX, centerY, width, length, color, rotation):

        rect = patches.Rectangle((centerX - length/2, centerY - width /2), length, width, color = color, alpha = 1)
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

    def speed(self):
        return math.sqrt(self.vel_x * self.vel_x + self.vel_y * self.vel_y)

    def __str__(self):
        return f"""Position: ({self.x}, {self.y})
                Velocity: ({self.vel_x}, {self.vel_y})
                Yaw: {self.yaw_angle}
                Steering: {self.steering_angle}"""

    def plot(self):
        # Model.ax.quiver(self.x, self.y, self.vel_x, self.vel_y, width=0.001, scale=1/(self.speed()*0.01))
        Model.ax.plot(self.x, self.y, 'o', color = Model.blue, alpha = 0.25)


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


if __name__ == "__main__":

    # Example state and differential
    state = State(0, 0, 2, 0, np.radians(0), np.radians(0))
    differentials = Differential(1,1,1)

    # Calculate and graph possible states 
    possible_states = Model.get_possible_states(state, differentials, False)

    plt.show()

