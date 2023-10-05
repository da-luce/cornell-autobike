from qagent import QAgent
import sys

sys.path.append("/usr/app/src")
sys.path.append("/usr/app/src/state_pred")
from state_pred import get_possible_states
import numpy as np


class BikeQAgent(QAgent):
    def __init__(self, alpha, gamma, rewards, dt):
        super().__init__(alpha, gamma, rewards, dt)

    def getPlayableActions(self, currentState, differentials, timestep):
        """Returns a list of states reachable from [currentState] after time [timestep]
        has elapsed. [currentState] is a list of 4 numbers: x coordinate, y coordinate,
        speed, and angle. [differentials] is also 4 numbers, but is the
        differences between cells in the matrix in SI units. [timestep] is what states are
        possible after [timestep] amount of time."""

        # To be replaced with issue #21
        STEERING_SAMPLING_DENSITY = 0.03  # Radians
        ACCELERATION_SAMPLING_DENSITY = 0.6  # m/s^2

        resolution = [ACCELERATION_SAMPLING_DENSITY, STEERING_SAMPLING_DENSITY]

        return get_possible_states(currentState, differentials, resolution)

    def getStateMatrix(self):
        """Returns a tuple, the first element is a matrix with dimensions: x coordinate,
        y coordinate, speed, angle. The second element is the differences between
        each element of the matrix in SI units. This function should be determined before
        compile-time based on the occupancy grid resolution and other physical factors.
        """
        return np.zeros((100, 100, 10, 10, 10, 10)), np.array(
            [0.2, 0.2, 0.2, 0.2, np.radians(0.01), np.radians(0.01)]
        )

    def set_up_rewards(self, end_state):
        rewards_new = np.copy(self.rewards)

        # Set goal to be very high reward
        rewards_new[end_state] = 999

        return rewards_new
