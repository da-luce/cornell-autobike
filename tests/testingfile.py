import unittest
import numpy as np
import sys
sys.path.append("/usr/app/src")

from qlearning import qagent
from qlearning import smallExampleQAgent

class TestQAgent(unittest.TestCase):
    def dummy_test():
        assert 1 > 0

    def setUp(self):
        self.alpha = 0.1
        self.gamma = 0.9
        self.rewards = np.zeros((9,))
        self.agent = smallExampleQAgent.SmallExampleQAgent(self.alpha, self.gamma, self.rewards)

    def test_q_learning(self):
        iterations = 1000
        end_state = 8

        self.agent.qlearning(self.rewards, iterations, end_state)
    
    def test_q_learning_2(self):
        iterations = 0
        end_state = 0

        self.agent.qlearning(self.rewards, iterations, end_state)
    
    def test_q_learning_3(self):
        rewards = np.random.rand(9,)
        iterations = 100
        end_state = 5

        self.agent.qlearning(rewards, iterations, end_state)
    
    def test_reset_matrix(self):
        iterations = 1000
        end_state = 8
        dimensions = 1

        self.agent.reset_matrix(self.rewards, iterations, end_state, dimensions)
        self.assertFalse(np.any(self.agent.q))
    
    def test_reset_matrix_2(self):
        rewards = np.random.rand(9,)
        iterations = 1000
        end_state = 8
        dimensions = 1

        self.agent.reset_matrix(rewards, iterations, end_state, dimensions)
        self.assertTrue(np.any(self.agent.q))
    
    def test_alter_matrix(self):
        iterations = 1000
        end_state = 8
        scale = 0.5

        self.agent.alter_matrix(self.rewards, iterations, end_state, scale)
        self.assertFalse(np.any(self.agent.q))
    
    def test_alter_matrix_2(self):
        rewards = np.random.rand(9,)
        iterations = 1000
        end_state = 8
        scale = 0.5

        self.agent.alter_matrix(rewards, iterations, end_state, scale)
        self.assertTrue(np.any(self.agent.q))

    def test_training(self):
        iterations = 100
        start_state = 0
        end_state = 8

        route = self.agent.training(start_state, end_state, iterations)
        print(route)
        #self.assertEqual(route[-1], end_state)


"""  
    def test_get_optimal_route(self):
        start_state = 0
        end_state = 8
        route = self.agent.get_optimal_route(start_state, end_state)
        self.assertTrue(len(route) > 0, "The route is empty.")
        print(f"Route: {route}")
        self.assertEqual(route[-1], end_state)

    def test_training(self):
        start_state, end_state, iterations = 0, 8, 100
        route = self.agent.training(start_state, end_state, iterations)
        self.assertEqual(route[-1], end_state)
"""


if __name__ == "__main__":
    unittest.main()