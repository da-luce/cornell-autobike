from subprocess import STARTF_USESTDHANDLES
import numpy as np
ITERATIONS = 1000


def set_up_rewards(end_location):
    rewards_new = np.copy(rewards)
    ending_state = location_to_state[end_location]

    # Set goal to be very high reward
    rewards_new[ending_state, ending_state] = 999

    return rewards_new, ending_state


def qlearning(rewards_new):
    q = getStateMatrix()

    for i in range(ITERATIONS):
        current_state = getRandomState()
        playable_actions = getPlayableActions(current_state)
        next_state = np.random.choice(playable_actions)
        temporal_difference = rewards_new[current_state, next_state] + gamma * \
            q[next_state, np.argmax(Q[next_state, ])] - \
            q[current_state, next_state]
        q[current_state, next_state] += alpha * temporal_difference

    return q


def traverse_q_matrix(start_location, end_location, q):
    route = [start_location]
    next_location = start_location
    while next_location != end_location:
        starting_state = location_to_state[start_location]
        next_state = np.argmax(q[starting_state])
        next_location = start_to_location[next_state]
        route.append(next_location)
        start_location = next_location

    return route


def get_optimal_route(start_location, end_location):
    rewards_new, ending_state = set_up_rewards(end_location)
    q = qlearning(rewards_new)
    route = traverse_q_matrix(start_location, end_location, q)
    return route
