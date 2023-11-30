import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import numpy as np
import matplotlib.pyplot as plt


def perlin(x, y, seed=0):
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


def sigmoid(x, threshold=0.5, steepness=10):
    """
    Sigmoid function for smooth thresholding.

    :param x: Input value or array.
    :param threshold: Center of the sigmoid function.
    :param steepness: Controls the steepness of the sigmoid curve.
    :return: Sigmoid function output.
    """
    return 1 / (1 + np.exp(-steepness * (x - threshold)))


def biased_easing(x, power=4):
    """
    Biased easing function using a power function to skew the distribution.

    :param x: Input value or array.
    :param power: Power to which the input value is raised.
    :return: Eased value.
    """
    return x**power


def generate_occupancy_grid(
    size=(100, 100), scale=2, threshold=0.5, steepness=100, time=0, seed=0
):
    """
    Generate a fake occupancy grid using Perlin noise.

    :param size: A tuple (width, height) specifying the size of the grid.
    :param scale: Scale factor for the Perlin noise.
    :param threshold: Threshold to convert the noise into a binary grid.
    :return: An array representing the occupancy grid.
    """
    # Create a grid of coordinates
    lin_x = np.linspace(0, 5, size[0], endpoint=False)
    lin_y = np.linspace(0, 5, size[1], endpoint=False)
    x, y = np.meshgrid(lin_x, lin_y)

    # Apply Perlin noise function (move upwards in time)
    noise = perlin(x / scale, y / scale + time, seed)

    # Normalize to 0-1
    noise = (noise - noise.min()) / (noise.max() - noise.min())

    # Apply biased easing function
    noise = biased_easing(noise, 5)

    return noise


def generate_occupancy_grid_sequence(
    size=(100, 100),
    scale=2,
    threshold=0,
    steepness=1,
    time_steps=1000,
    speed_offset=100,
    seed=0,
):
    """
    Generate a sequence of occupancy grids over time using Perlin noise with soft threshold.
    """
    grids = []
    for t in range(time_steps):
        t = t / speed_offset
        grid = generate_occupancy_grid(size, scale, threshold, steepness, t, seed)
        grids.append(grid)
    return grids


def overlay_grids(grids_list):
    """
    Overlay multiple grids by averaging their values.

    :param grids_list: List of grids to be overlaid.
    :return: A single grid representing the overlay of all input grids.
    """
    # Sum all the grids
    combined_grid = sum(grids_list)

    # Normalize the combined grid
    combined_grid /= len(grids_list)

    return combined_grid


def combine_sequences(sequence_list):
    """
    Combine multiple grid sequences into a single sequence by overlaying corresponding grids.

    :param sequence_list: List of grid sequences to be combined.
    :return: A single grid sequence representing the overlay of all input sequences.
    """
    combined_sequence = []
    time_steps = len(sequence_list[0])  # Assuming all sequences have the same length

    for t in range(time_steps):
        # Collect the grid at time step 't' from each sequence
        grids_at_t = [sequence[t] for sequence in sequence_list]

        # Overlay these grids
        combined_grid = overlay_grids(grids_at_t)

        # Append the combined grid to the sequence
        combined_sequence.append(combined_grid)

    return combined_sequence


# Generate and plot the occupancy grid with Perlin noise
# occupancy_grid_perlin = generate_occupancy_grid_with_perlin_noise()
# plt.imshow(occupancy_grid_perlin, cmap="gray")
# plt.title("Occupancy Grid with Perlin Noise")
# plt.show()

# Generate a sequence of grids
grid_sequence1 = generate_occupancy_grid_sequence(speed_offset=10000, seed=0)
grid_sequence2 = generate_occupancy_grid_sequence(speed_offset=80, seed=2000)
grid_sequence3 = generate_occupancy_grid_sequence(speed_offset=100, seed=400)

overlayed = combine_sequences([grid_sequence1, grid_sequence2, grid_sequence3])

# Create an animation
fig, ax = plt.subplots()
ims = []
for grid in overlayed:
    im = ax.imshow(grid, animated=True, cmap="gray")
    ims.append([im])

ani = animation.ArtistAnimation(fig, ims, interval=20, blit=True)
plt.show()
