"""Module representing a situation where Q-Learning can be used."""

import math
from abc import abstractmethod

import numpy as np


class QAgent:
    """Class representing a situation where Q-Learning can be used."""

    @abstractmethod
    def get_playable_actions(self, current_state, differentials, timestep):
        """Returns a list of states reachable from [current_state] after time [timestep]
        has elapsed. [current_state] is a list of 4 numbers: x coordinate, y coordinate,
        speed, and angle. [differentials] is also 4 numbers, but is the
        differences between cells in the matrix in SI units. [timestep] is what states are
        possible after [timestep] amount of time."""
        print(current_state, differentials, timestep)

    @abstractmethod
    def get_state_matrix(self):
        """Returns a tuple, the first element is a matrix with dimensions: x coordinate,
        y coordinate, speed, angle. The second element is the differences between
        each element of the matrix in SI units. This function should be determined before
        compile-time based on the occupancy grid resolution and other physical factors.
        """

    @abstractmethod
    def set_up_rewards(self, end_state):
        """Creates the reward matrix for the QAgent."""
        print(end_state)

    def get_random_state(self):
        """Selects a random state from all states in the state space."""
        result = []
        for dim in self.q.shape:
            result.append(np.random.randint(0, high=dim))
        return tuple(result)

    def get_rewards(self, occupancy_grid, distance):
        """Returns the reward matrix given an occupancy grid and the current distance to
        the goal."""
        constant_a = 1
        constant_b = 4
        constant_c = 2

        # The reward should increase as we approach the goal and
        # decrease as the probability of encountering an object increases.

        rewards = (constant_a / math.sqrt((distance**2) + constant_b)) * (
            1 - (constant_c * occupancy_grid)
        )

        return rewards

    def qlearning(self, rewards_new, iterations, end_state):
        """Fill in the Q-matrix"""
        for _ in range(iterations):
            current_state = self.get_random_state()
            if current_state == end_state:
                continue
            playable_actions = self.get_playable_actions(
                current_state, self.differentials, self.dt
            )
            temporal_difference = (
                rewards_new[current_state]
                + self.gamma * np.amax(self.q[playable_actions])
                - self.q[current_state]
            )
            self.q[current_state] += self.alpha * temporal_difference

    def reset_matrix(self, rewards_new, iterations, end_state, dimensions):
        """Reset the Q-matrix, and rerun the Q-learning algorithm"""
        shape = tuple([len(self.q)] * dimensions)
        self.q: np.ndarray = np.zeros(shape)
        QAgent.qlearning(self, rewards_new, iterations, end_state)

    def alter_matrix(self, rewards_new, iterations, end_state, scale):
        """Partially reset the Q-matrix keeping a scale fraction of the previous
        Q-matrix as a starting place, and rerun the Q-learning algorithm"""
        rewards_new = rewards_new * scale
        QAgent.qlearning(self, rewards_new, iterations, end_state)

    def get_optimal_route(self, start_state, end_state):
        """Given a Q-matrix, greedily select the highest next Q-matrix value until the
        end state is reached. This assumes that greedily choosing the next path will
        eventually reach the finish, which is not always true. This is used as a
        visualization, but not a step in the navigation algorithm."""
        route = [start_state]
        next_state = start_state
        while next_state != end_state:
            playable_actions = self.get_playable_actions(
                next_state, self.differentials, self.dt
            )
            next_state = playable_actions[0][np.argmax(self.q[playable_actions])]
            if next_state in route:
                route.append(next_state)
                break
            route.append(next_state)

        return route

    def training(self, start_state, end_state, iterations):
        """Run all the steps of the Q-learning algorithm."""
        rewards_new = self.set_up_rewards(end_state)
        self.qlearning(rewards_new, iterations, end_state)
        route = self.get_optimal_route(start_state, end_state)
        return route

    def __init__(self, alpha, gamma, rewards, dt):
        self.gamma = gamma
        self.alpha = alpha
        self.rewards = rewards
        self.q, self.differentials = self.get_state_matrix()
        self.dt = dt
