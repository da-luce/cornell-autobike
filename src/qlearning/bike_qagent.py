"""Module which implements QAgent for the dynamics of the bike."""

import numpy as np

from src.qlearning.qagent import QAgent


class BikeQAgent(QAgent):
    """Class which implements QAgent for the dynamics of the bike."""

    def get_playable_actions(self, current_state, differentials, timestep):
        print(current_state, differentials, timestep)

        raise NotImplementedError

    def get_state_matrix(self):
        return np.zeros((1000, 1000, 10000, 10000, 360))

    def set_up_rewards(self, end_state):
        rewards_new = np.copy(self.rewards)

        # Set goal to be very high reward
        rewards_new[end_state] = 999

        return rewards_new
