import numpy as np
import sys
sys.path.append("/usr/app/src")

from qlearning import qagent
from qlearning import smallExampleQAgent

def dummy_test():
    assert 1 > 0

def q_matrix_test():
    bike_sample = smallExampleQAgent.SmallExampleQAgent(0.1, 0.9, np.zeros((9,)))
    print(qagent.QAgent.qlearning(bike_sample, np.zeros((9,)), 1000, 10))

def reset_matrix_test():
    bike_sample = SmallExampleQAgent(0.1, 0.9, np.zeros((9,)))
    print(QAgent.reset_matrix(bike_sample, np.zeros((9,)), 1000, 10))

def alter_matrix_test():
    bike_sample = SmallExampleQAgent(0.1, 0.9, np.zeros((9,)))
    print(QAgent.alter_matrix(bike_sample, np.zeros((9,)), 1000, 100))

if __name__ == "__main__":
    dummy_test()
    q_matrix_test()
    reset_matrix_test()