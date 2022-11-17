import qagent as model
import numpy as np

if __name__ == "__main__":
    qagent = model.QAgent(0.1, 0.9, np.zeros((9,)), 1)
    route = qagent.training(0, 8, 1000000)
    print(route)
