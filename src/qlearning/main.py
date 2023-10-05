import smallExampleQAgent
import bikeQAgent
import numpy as np

if __name__ == "__main__":
    if False:
        qagent = smallExampleQAgent.SmallExampleQAgent(0.1, 0.9, np.zeros((9,)))
        route = qagent.training(0, 8, 100000)
        print(route)
    if True:
        qagent = bikeQAgent.BikeQAgent(
            0.1,
            0.9,
            np.zeros((100, 100, 10, 10, 10, 10)),
        )
        route = qagent.training(0, 8, 100000)
        print(route)
