import numpy as np
import cv2
import matplotlib.pyplot as plt
from src.grid_pred.grid_gen import gen_grid

# Occupancy grid
SIZE_X = 128
SIZE_Y = 128
TIMESTEP = 0.3

# From OpenCV documentation:
# "The function finds an optical flow for each prev pixel using the [67]
# algorithm so that
# prev(y,x) = next(y + flow(y,x)[1], x + flow(y,x)[0])"
#


# Function to plot optical flow vectors
def draw_flow(img, flow, step=4, scale=10):
    """
    Draw optical flow vectors on the image.
    :param img: Grayscale image to draw on.
    :param flow: Optical flow vectors (shape: height x width x 2).
    :param step: Space between vectors. Lower values mean more density.
    :param scale: Scale factor for visualizing the flow vectors.
    """

    # Create a figure
    plt.figure(figsize=(10, 10))

    i, j = 10, 10  # Example pixel coordinates
    flow_vector = flow[i, j]

    # Display the image
    plt.imshow(img, cmap="gray", interpolation="nearest")
    plt.title("Optical Flow Vectors")
    plt.axis("off")

    # Create a meshgrid for the flow
    h, w = img.shape[:2]
    y, x = np.mgrid[0:h:step, 0:w:step].reshape(2, -1)
    fx, fy = flow[y, x].T * scale

    # Plot the flow vectors
    plt.quiver(
        x,
        y,
        fx,
        fy,
        color="r",
        angles="xy",
        scale_units="xy",
        scale=1,
        headwidth=3,
        headlength=4,
    )

    plt.show()


def draw_dense_quiver_flow(img, flow, step=1, scale=1e6):
    """
    Draw dense optical flow vectors using quiver plot.
    :param img: Grayscale image to draw on.
    :param flow: Optical flow vectors (shape: height x width x 2).
    :param step: Space between vectors. Lower values mean more density.
    :param scale: Scale factor for visualizing the flow vectors.
    """
    # Create a figure for plotting
    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    ax.imshow(img, cmap="gray", interpolation="nearest")
    ax.set_title("Optical Flow Vectors")
    ax.axis("off")

    # Generate a grid of points
    h, w = img.shape
    y, x = np.mgrid[step / 2 : h : step, step / 2 : w : step].reshape(2, -1).astype(int)
    fx, fy = flow[y, x].T

    # Plotting the flow vectors
    ax.quiver(
        x, y, fx, fy, color="r", angles="xy", scale_units="xy", scale=scale, width=0.005
    )

    return fig


def scale_flow_vectors(flow):
    """
    Scale the optical flow vectors so that the smallest non-zero vector magnitude becomes 1.

    :param flow: A NumPy array of shape (height, width, 2) containing the flow vectors.
    :return: A NumPy array of the scaled flow vectors.
    """
    # Calculate magnitudes of the flow vectors
    magnitudes = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)

    # Find the smallest non-zero magnitude
    min_nonzero_magnitude = np.min(magnitudes[np.nonzero(magnitudes)])

    # Avoid division by zero in case all vectors are zero
    if min_nonzero_magnitude == 0:
        return flow  # Return original flow if all vectors are zero

    # Calculate scaling factor
    scaling_factor = 1 / min_nonzero_magnitude

    # Scale the flow vectors
    scaled_flow = flow * scaling_factor

    return scaled_flow


def predict_grid(image2, flow):
    """
    Predict the next grid state based on the optical flow vectors.

    :param image2: The second image (numpy array) from which to predict the next state.
    :param flow: Optical flow vectors obtained between the first and second images.
    :return: The predicted third image as a numpy array.
    """
    # Initialize an empty array for the predicted image
    h, w = image2.shape[:2]
    predicted_image = np.zeros_like(image2)

    # For each pixel in image2, move it according to the flow vector
    for y in range(h):
        for x in range(w):
            flow_x, flow_y = flow[y, x]
            new_x, new_y = x + np.floor(flow_x * TIMESTEP), y + np.floor(
                flow_y * TIMESTEP
            )

            # Convert new_x and new_y to integers for use as indices
            new_x, new_y = int(new_x), int(new_y)

            # Make sure the new coordinates are within image bounds
            if 0 <= new_x < w and 0 <= new_y < h:
                predicted_image[new_y, new_x] = image2[y, x]

    # Smooth and return
    predicted_image = cv2.blur(predicted_image, (5, 5))
    return predicted_image


if __name__ == "__main__":

    frameA = gen_grid(SIZE_X, SIZE_Y, time=0)
    frameB = gen_grid(SIZE_X, SIZE_Y, time=0 + TIMESTEP)
    flow = cv2.calcOpticalFlowFarneback(
        frameA,
        frameB,
        None,
        pyr_scale=0.5,
        levels=10,
        winsize=64,
        iterations=100,
        poly_n=10,
        poly_sigma=1.2,
        flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
    )

    flow = flow * 10e8

    flow.astype("int16").tofile("./temp")

    plt.figure("Frame A")
    plt.imshow(frameA, cmap="gray")

    plt.figure("Frame B")
    plt.imshow(frameB, cmap="gray")

    frameC = predict_grid(frameB, flow)
    plt.figure("Frame C")
    plt.imshow(frameC, cmap="gray")

    flows = draw_dense_quiver_flow(frameA, flow)

    plt.show()
