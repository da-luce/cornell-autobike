"""Module which runs the desired QAgent."""

import numpy as np

from src.qlearning import small_example_qagent

if __name__ == "__main__":
    qagent = small_example_qagent.SmallExampleQAgent(0.1, 0.9, np.zeros((9,)))
    route = qagent.training(0, 8, 100000)
    print(route)
