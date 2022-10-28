import numpy as np
from HardToWriteFunctions import getPlayableActions, getStateMatrix


class QAgent():
    def getRandomState(self):
        result = []
        for i in range(len(self.q.shape)):
            result.append(np.random.randint(0, high=np.max(self.q, axis=i)))
        return tuple(result)

    def set_up_rewards(self, end_state):
        rewards_new = np.copy(self.rewards)

        # Set goal to be very high reward
        rewards_new[end_state] = 999

        return rewards_new

    def qlearning(self, rewards_new, iterations):
        for _ in range(iterations):
            current_state = self.getRandomState()
            playable_actions = getPlayableActions(
                current_state, self.differentials, self.dt)
            next_state = np.random.choice(playable_actions)
            temporal_difference = rewards_new[current_state] + self.gamma * \
                self.q[np.argmax(self.q[playable_actions])] - \
                self.q[current_state]
            self.q[current_state] += self.alpha * \
                temporal_difference

    def get_optimal_route(self, start_state, end_state):
        route = [start_state]
        next_state = start_state
        while next_state != end_state:
            playable_actions = getPlayableActions(
                next_state, self.differentials, self.dt)
            next_state = np.argmax(self.q[playable_actions])
            route.append(next_state)

        return route

    def training(self, start_state, end_state, iterations):
        rewards_new = self.set_up_rewards(end_state)
        self.qlearning(rewards_new, iterations)
        route = self.get_optimal_route(start_state, end_state)
        return route

    def __init__(self, alpha, gamma, actions, rewards, dt):
        self.gamma = gamma
        self.alpha = alpha
        self.actions = actions
        self.rewards = rewards
        self.q, self.differentials = getStateMatrix()
        self.dt = dt
