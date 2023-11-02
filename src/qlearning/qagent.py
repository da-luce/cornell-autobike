import numpy as np
from abc import abstractmethod


class QAgent:
    @abstractmethod
    def getPlayableActions(self, currentState, differentials):
        pass

    @abstractmethod
    def getStateMatrix(self):
        pass

    @abstractmethod
    def set_up_rewards(self, end_state):
        pass

    def getRandomState(self):
        result = []
        for i in range(len(self.q.shape)):
            result.append(np.random.randint(0, high=self.q.shape[i]))
        return np.array(result)

    def qlearning(self, rewards_new, iterations, end_state):
        for _ in range(iterations):
            current_state = self.getRandomState()
            if np.all(current_state == end_state):
                continue
            playable_actions = self.getPlayableActions(
                current_state, self.differentials
            )
            temporal_difference = (
                rewards_new[current_state]
                + self.gamma * np.amax(self.q[playable_actions])
                - self.q[current_state]
            )
            self.q[current_state] += self.alpha * temporal_difference

    def get_optimal_route(self, start_state, end_state):
        route = [start_state]
        next_state = start_state
        while next_state != end_state:
            playable_actions = self.getPlayableActions(next_state, self.differentials)
            t1 = self.q[playable_actions]
            t2 = np.argmax(self.q[playable_actions])
            t3 = playable_actions[0]
            next_state = playable_actions[0][np.argmax(self.q[playable_actions])]
            route.append(next_state)

        return route

    def training(self, start_state, end_state, iterations):
        rewards_new = self.set_up_rewards(end_state)
        self.qlearning(rewards_new, iterations, end_state)
        print(self.q.reshape((3, 3)))
        route = self.get_optimal_route(start_state, end_state)
        return route

    def __init__(self, alpha, gamma, rewards):
        self.gamma = gamma
        self.alpha = alpha
        self.rewards = rewards
        self.q, self.differentials = self.getStateMatrix()
