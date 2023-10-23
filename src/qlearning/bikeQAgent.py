from qagent import QAgent
import sys

sys.path.append("/usr/app/src")

from state_pred import sim
import numpy as np


class BikeQAgent(QAgent):
    def __init__(self, alpha, gamma, rewards):
        super().__init__(alpha, gamma, rewards)

    def getPlayableActions(self, currentState, differentials):
        # To be replaced with issue #21
        STEERING_SAMPLING_DENSITY = 0.03  # Radians
        ACCELERATION_SAMPLING_DENSITY = 0.6  # m/s^2

        resolution = np.array(
            [ACCELERATION_SAMPLING_DENSITY, STEERING_SAMPLING_DENSITY]
        )

        return sim.get_possible_states(currentState, differentials, resolution)

    def getStateMatrix(self):
        return np.zeros((100, 100, 10, 10, 10, 10)), np.array(
            [0.2, 0.2, 0.2, 0.2, np.radians(0.01), np.radians(0.01)]
        )

    def set_up_rewards(self, end_state):
        rewards_new = np.copy(self.rewards)

        # Set goal to be very high reward
        rewards_new[end_state] = 999

        return rewards_new
