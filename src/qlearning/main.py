import smallExampleQAgent
import numpy as np
import settings
import send_data
import receive_data
import time

if __name__ == "__main__":
    settings.init()
    settings.gps = 0
    settings.angle = 0
    while (True):
        send_data.init()
        receive_data.init()
        qagent = smallExampleQAgent.SmallExampleQAgent(0.1, 0.9, np.zeros((9,)))
        route = qagent.training(0, 8, 100000)
        print(route)
        print("Angle " + str(settings.angle))
        print("GPS " + str(settings.gps))
        time.sleep(1)
