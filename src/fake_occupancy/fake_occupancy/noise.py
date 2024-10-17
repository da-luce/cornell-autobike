"""
Perlin and other noise related functions. FIXME: stolen from where?
"""

import numpy as np


def perlin(x: float, y: float, seed: int = 0) -> float:
    def lerp(a, b, x):
        "linear interpolation"
        return a + x * (b - a)

    def fade(t):
        "6t^5 - 15t^4 + 10t^3"
        return 6 * t**5 - 15 * t**4 + 10 * t**3

    def gradient(h, x, y):
        "grad converts h to the right gradient vector and return the dot product with (x,y)"
        vectors = np.array([[0, 1], [0, -1], [1, 0], [-1, 0]])
        g = vectors[h % 4]
        return g[:, :, 0] * x + g[:, :, 1] * y

    # permutation table
    np.random.seed(seed)
    p = np.arange(256, dtype=int)
    np.random.shuffle(p)
    p = np.stack([p, p]).flatten()
    # coordinates of the top-left
    xi, yi = x.astype(int), y.astype(int)
    # internal coordinates
    xf, yf = x - xi, y - yi
    # fade factors
    u, v = fade(xf), fade(yf)
    # noise components
    n00 = gradient(p[p[xi] + yi], xf, yf)
    n01 = gradient(p[p[xi] + yi + 1], xf, yf - 1)
    n11 = gradient(p[p[xi + 1] + yi + 1], xf - 1, yf - 1)
    n10 = gradient(p[p[xi + 1] + yi], xf - 1, yf)
    # combine noises
    x1 = lerp(n00, n10, u)
    x2 = lerp(n01, n11, u)  # FIX1: I was using n10 instead of n01
    return lerp(x1, x2, v)  # FIX2: I also had to reverse x1 and x2 here


def sigmoid(x: float, threshold: float = 0.5, steepness: float = 10) -> float:
    """
    Sigmoid function for smooth thresholding.

    :param x: Input value or array.
    :param threshold: Center of the sigmoid function.
    :param steepness: Controls the steepness of the sigmoid curve.
    :return: Sigmoid function output.
    """
    return 1 / (1 + np.exp(-steepness * (x - threshold)))


def biased_easing(x: float, power: int = 4) -> float:
    """
    Biased easing function using a power function to skew the distribution.

    :param x: Input value or array.
    :param power: Power to which the input value is raised.
    :return: Eased value.
    """
    return x**power
