import numpy as np
from abc import abstractmethod
import math


class QAgent():
    @abstractmethod
    def getPlayableActions(self, currentState, differentials, timestep):
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
        return tuple(result)

    def qlearning(self, rewards_new, iterations, end_state):
        for _ in range(iterations):
            current_state = self.getRandomState()
            if current_state == end_state:
                continue
            playable_actions = self.getPlayableActions(
                current_state, self.differentials, self.dt)
            temporal_difference = rewards_new[current_state] + self.gamma * \
                np.amax(self.q[playable_actions]) - \
                self.q[current_state]
            self.q[current_state] += self.alpha * \
                temporal_difference

    def get_optimal_route(self, start_state, end_state):
        route = [start_state]
        next_state = start_state
        while next_state != end_state:
            playable_actions = self.getPlayableActions(
                next_state, self.differentials, self.dt)
            t1 = self.q[playable_actions]
            t2 = np.argmax(self.q[playable_actions])
            t3 = playable_actions[0]
            next_state = playable_actions[0][np.argmax(
                self.q[playable_actions])]
            route.append(next_state)

        return route

    def training(self, start_state, end_state, iterations):
        rewards_new = self.set_up_rewards(end_state)
        self.qlearning(rewards_new, iterations, end_state)
        print(self.q.reshape((3, 3)))
        route = self.get_optimal_route(start_state, end_state)
        return route

    def __init__(self, alpha, gamma, rewards, dt):
        self.gamma = gamma
        self.alpha = alpha
        self.rewards = rewards
        self.q, self.differentials = self.getStateMatrix()
        self.dt = dt


class SmallExampleQAgent(QAgent):
    def __init__(self, alpha, gamma, rewards):
        super().__init__(alpha, gamma, rewards, 1)

    def getPlayableActions(self, currentState, differentials, timestep):
        playableActions = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0],
                                    [1, 0, 1, 0, 1, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0, 1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, 0, 0],
                                    [0, 1, 0, 0, 0, 0, 0, 1, 0],
                                    [0, 0, 1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 1, 0, 1, 0, 1],
                                    [0, 0, 0, 0, 0, 0, 0, 1, 0]])

        return np.nonzero(playableActions[currentState])

    def getStateMatrix():
        return np.zeros((9,)), (1,)

    def set_up_rewards(self, end_state):
        rewards_new = np.copy(self.rewards)

        # Set goal to be very high reward
        rewards_new[end_state] = 999

        return rewards_new


class BikeQAgent(QAgent):
    def __init__(self, alpha, gamma, rewards, dt):
        super().__init__(alpha, gamma, rewards, dt)

    def getPlayableActions(currentState, differentials, timestep):
        """Returns a list of states reachable from [currentState] after time [timestep]
        has elapsed. [currentState] is a list of 4 numbers: x coordinate, y coordinate,
        speed, and angle. [differentials] is also 4 numbers, but is the
        differences between cells in the matrix in SI units. [timestep] is what states are
        possible after [timestep] amount of time."""
        acceleration_power = 1  # m/s/s
        braking_power = 1  # m/s/s
        max_turning_rate = 30  # deg/s

        # calculating max velocity reachable
        max_vel = currentState[2] + acceleration_power*(timestep)

        # calculating min velocity reachable
        min_vel = currentState[2] - braking_power*(timestep)

        # calculating max clockwise angle reachable
        clock_max_angle = (currentState[3]-(max_turning_rate*timestep)) % 360

        # calculating max counter-clockwise angle reachable
        counter_max_angle = (currentState[3]+(max_turning_rate*timestep)) % 360

        # min_x_coordinate
        init_x_coord = currentState[0]

        # min_y_coordinate
        init_y_coord = currentState[1]

        # calculating max displacement possible if continuing on same path with max acceleration
        max_S = currentState[2]*timestep + 0.5 * \
            acceleration_power*timestep*timestep

        # calculating max change in x coordinate
        max_x_coord = init_x_coord + (max_S * math.cos(currentState[3]))

        # calculating max change in y coordinate
        max_y_coord = init_y_coord + (max_S * math.sin(currentState[3]))

        angle_diff_check = counter_max_angle - (clock_max_angle-360)

        """while angle_diff_check/differentials[3] > 0:
            x1 = clock_max_angle/differentials[3] +
            return [[[(x, y, z, x1) for x in range(init_x_coord/differentials[0], max_x_coord/differentials[0])]
            for y in range(init_y_coord/differentials[1], max_y_coord/differentials[1])]
            for z in range(min_vel/differentials[2], max_vel/differentials[2])]

            for x1 in range(clock_max_angle/differentials[3], counter_max_angle/differentials[3])]

            angle_diff_check/differentials[3] -= 1"""

        raise NotImplementedError

    def getStateMatrix():
        """Returns a tuple, the first element is a matrix with dimensions: x coordinate,
        y coordinate, speed, angle. The second element is the differences between
        each element of the matrix in SI units. This function should be determined before
        compile-time based on the occupancy grid resolution and other physical factors."""
        return np.zeros((1000, 1000, 10000, 10000, 360))

    def set_up_rewards(self, end_state):
        rewards_new = np.copy(self.rewards)

        # Set goal to be very high reward
        rewards_new[end_state] = 999

        return rewards_new
