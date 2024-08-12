import numpy as np

from src.qlearning import qagent


class SmallExampleQAgent(qagent.QAgent):
    def __init__(self, alpha, gamma, rewards):
        super().__init__(alpha, gamma, rewards, 1)

    def getPlayableActions(self, current_state, differentials, timestep):
        playable_actions = np.array(
            [
                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                [1, 0, 1, 0, 1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1, 0, 1, 0, 1],
                [0, 0, 0, 0, 0, 0, 0, 1, 0],
            ]
        )

        return np.nonzero(playable_actions[current_state])

    def get_state_matrix(self):
        return np.zeros((9,)), (1,)

    def set_up_rewards(self, end_state):
        rewards_new = np.copy(self.rewards)

        # Set goal to be very high reward
        rewards_new[end_state] = 999

        return rewards_new
