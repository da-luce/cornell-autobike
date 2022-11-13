import qagent as model

if __name__ == "__main__":
    qagent = model.QAgent(0.1, 0.9, )
    route = qagent.training()
    print(route)
