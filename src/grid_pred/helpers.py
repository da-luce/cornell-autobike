import numpy as np


def sigmoid(x, alpha: float = 1.0, beta: float = 0.0):
    """
    Sigmoid function, returns value in (0, 1)
    alpha: exponential scaling factor, larger alpha results in steeper function
    beta: horizontal shift, i.e. where the function returns 0.5
    """

    return 1 / (1 + np.exp(-alpha * (x - beta)))
