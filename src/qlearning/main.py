import smallExampleQAgent
import numpy as np

if __name__ == "__main__":
    qagent = smallExampleQAgent.SmallExampleQAgent(0.1, 0.9, np.zeros((9,)))
    route = qagent.training(0, 8, 100000)
    print(route)
