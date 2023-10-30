import numpy as np
import sys
sys.path.append("/usr/app/src")

from qlearning import qagent
from qlearning import smallExampleQAgent

def dummy_test():
    assert 1 > 0

def q_matrix_test():
    bike_sample = smallExampleQAgent.SmallExampleQAgent(0.1, 0.9, np.zeros((10,10)))
    qagent.QAgent.qlearning(bike_sample, np.zeros((10,)), 1000, 10)
    print(bike_sample)

def reset_matrix_test():
    bike_sample = smallExampleQAgent.SmallExampleQAgent(0.1, 0.9, np.zeros((10,10)))
    qagent.QAgent.reset_matrix(bike_sample, np.zeros((10,)), 1000, 5, 1)
    print(bike_sample)

def alter_matrix_test():
    bike_sample = smallExampleQAgent.SmallExampleQAgent(0.1, 0.9, np.zeros((10,10)))
    qagent.QAgent.alter_matrix(bike_sample, np.zeros((10,)), 1000, 10)
    print(bike_sample)

if __name__ == "__main__":
    dummy_test()
    q_matrix_test()
    reset_matrix_test()
    alter_matrix_test()