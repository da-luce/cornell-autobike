import numpy as np
import pytest

from src.qlearning import smallExampleQAgent


@pytest.fixture
def agent_setup():
    alpha = 0.1
    gamma = 0.9
    rewards = np.zeros((9,))
    agent = smallExampleQAgent.SmallExampleQAgent(alpha, gamma, rewards)
    return agent, rewards


def test_q_learning(agent_setup):
    agent, rewards = agent_setup
    iterations = 1000
    end_state = 8

    agent.qlearning(rewards, iterations, end_state)


def test_q_learning_2(agent_setup):
    agent, rewards = agent_setup
    iterations = 0
    end_state = 0

    agent.qlearning(rewards, iterations, end_state)


def test_q_learning_3(agent_setup):
    agent, _ = agent_setup
    rewards = np.random.rand(9)
    iterations = 1000
    end_state = 5

    agent.qlearning(rewards, iterations, end_state)


def test_reset_matrix(agent_setup):
    agent, rewards = agent_setup
    iterations = 1000
    end_state = 8
    dimensions = 1

    agent.reset_matrix(rewards, iterations, end_state, dimensions)
    assert not np.any(agent.q)


def test_reset_matrix_2(agent_setup):
    agent, _ = agent_setup
    rewards = np.random.rand(9)
    iterations = 1000
    end_state = 8
    dimensions = 1

    agent.reset_matrix(rewards, iterations, end_state, dimensions)
    assert np.any(agent.q)


def test_reset_matrix_3(agent_setup):
    agent, _ = agent_setup
    rewards = np.random.rand(9)
    iterations = 1000
    end_state = 1
    dimensions = 1

    agent.reset_matrix(rewards, iterations, end_state, dimensions)
    assert np.any(agent.q)


def test_alter_matrix(agent_setup):
    agent, rewards = agent_setup
    iterations = 1000
    end_state = 8
    scale = 0.5

    agent.alter_matrix(rewards, iterations, end_state, scale)
    assert not np.any(agent.q)


def test_alter_matrix_2(agent_setup):
    agent, _ = agent_setup
    rewards = np.random.rand(9)
    iterations = 1000
    end_state = 8
    scale = 0.5

    agent.alter_matrix(rewards, iterations, end_state, scale)
    assert np.any(agent.q)


def test_alter_matrix_3(agent_setup):
    agent, _ = agent_setup
    rewards = np.random.rand(9)
    iterations = 1000
    end_state = 5
    scale = 0.1

    agent.alter_matrix(rewards, iterations, end_state, scale)
    assert np.any(agent.q)


def test_training(agent_setup):
    agent, _ = agent_setup
    iterations = 1000
    start_state = 0
    end_state = 8

    route = agent.training(start_state, end_state, iterations)
    print(route)


def test_training2(agent_setup):
    agent, _ = agent_setup
    iterations = 1000
    start_state = 1
    end_state = 5

    route = agent.training(start_state, end_state, iterations)
    print(route)


def test_get_optimal_route(agent_setup):
    agent, _ = agent_setup
    start_state = 0
    end_state = 8
    route = agent.get_optimal_route(start_state, end_state)
    assert len(route) > 0, "The route is empty."
    print(f"Route: {route}")


def test_get_rewards_1(agent_setup):
    agent, _ = agent_setup
    rewards = agent.get_rewards(occupancy_grid=np.random.rand(9), distance=2)
    print(f"Rewards: {rewards}")


def test_get_rewards_2(agent_setup):
    agent, _ = agent_setup
    rewards = agent.get_rewards(occupancy_grid=np.zeros((9,)), distance=2)
    print(f"Rewards: {rewards}")


def test_get_rewards_3(agent_setup):
    agent, _ = agent_setup
    rewards = agent.get_rewards(occupancy_grid=np.random.rand(9), distance=0)
    print(f"Rewards: {rewards}")


def test_get_rewards_4(agent_setup):
    agent, _ = agent_setup
    rewards = agent.get_rewards(occupancy_grid=np.zeros((9,)), distance=0)
    print(f"Rewards: {rewards}")


def test_get_rewards_5(agent_setup):
    agent, _ = agent_setup
    rewards = agent.get_rewards(occupancy_grid=np.random.rand(9), distance=200)
    print(f"Rewards: {rewards}")
