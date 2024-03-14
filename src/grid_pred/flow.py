import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.grid_pred.grid_gen import gen_grid

# Occupancy grid
SIZE_X = 128
SIZE_Y = 128
TIMESTEP = 0.3

import numpy as np


def add_zero_border(image, border_size):
    """
    Adds a border of 0.0 values around the image.

    Parameters:
    - image: NumPy array of the image.
    - border_size: Integer or tuple specifying the size of the border. If an integer is provided,
                   the same border size is used on all sides. If a tuple of four integers is provided,
                   they specify the size of the border on the top, bottom, left, and right, respectively.

    Returns:
    - A new image with the added border.
    """

    # If border_size is a single integer, convert it to a tuple (top, bottom, left, right)
    if isinstance(border_size, int):
        border_size = (border_size, border_size, border_size, border_size)

    # Unpack the border sizes
    top, bottom, left, right = border_size

    # Get the shape of the original image
    original_height, original_width = image.shape[:2]

    # Calculate the shape of the new image
    new_height = original_height + top + bottom
    new_width = original_width + left + right

    # Create a new image filled with zeros
    if image.ndim == 2:  # Grayscale image
        new_image = np.zeros((new_height, new_width), dtype=image.dtype)
    else:  # Color image
        new_image = np.zeros((new_height, new_width, image.shape[2]), dtype=image.dtype)

    # Copy the original image into the center of the new image
    new_image[top : top + original_height, left : left + original_width] = image

    return new_image


# From OpenCV documentation:
# "The function finds an optical flow for each prev pixel using the [67]
# algorithm so that
# prev(y,x) = next(y + flow(y,x)[1], x + flow(y,x)[0])"


def draw_flow_heatmap(flow):
    """
    Visualize the magnitudes of optical flow vectors as a heatmap.
    :param flow: Optical flow vectors (shape: height x width x 2).
    """
    # Calculate the magnitude of flow vectors
    magnitude = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)

    # Normalize the magnitude to the range [0, 1] for displaying
    normalized_magnitude = cv2.normalize(magnitude, None, 0, 1, cv2.NORM_MINMAX)

    # Create a figure
    plt.figure(figsize=(6.4, 4.8))

    # Display the heatmap of magnitudes
    plt.imshow(normalized_magnitude, cmap="inferno")

    plt.title("Optical Flow Heatmap")
    plt.axis("off")
    plt.colorbar(label="Flow Magnitude")
    plt.show()


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


def scale_flow_vectors(flow, desired_scale=1):
    """
    Scale the optical flow vectors so that the smallest non-zero vector magnitude becomes the desired scale.

    :param flow: A NumPy array of shape (height, width, 2) containing the flow vectors.
    :param desired_scale: The desired scale for the smallest non-zero vector magnitude.
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
    scaling_factor = desired_scale / min_nonzero_magnitude

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


def predict_grid_modified(image2, flow, TIMESTEP=0.3):
    """
    Predict the next grid state based on the optical flow vectors, adding pixel
    values to the new locations.

    :param image2: The second image (numpy array) from which to predict the next state.
    :param flow: Optical flow vectors obtained between the first and second images.
    :param TIMESTEP: The time step used for scaling the flow vectors.
    :return: The predicted third image as a numpy array.
    """
    # Initialize an empty array for the predicted image
    h, w = image2.shape[:2]
    predicted_image = np.zeros_like(
        image2, dtype=np.float32
    )  # Use float32 to prevent overflow during addition

    # For each pixel in image2, add it according to the flow vector
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
                predicted_image[new_y, new_x] += image2[
                    y, x
                ]  # Add the value to the new location

    # Clip values to ensure they remain within valid range after addition
    predicted_image = np.clip(predicted_image, 0, 255)

    # Optionally convert predicted_image back to original data type (e.g., uint8)
    predicted_image = predicted_image.astype(image2.dtype)

    # Smooth and return
    predicted_image = cv2.blur(predicted_image, (5, 5))
    return predicted_image


def add_contrast(image, alpha=1.0):
    """
    Enhance contrast of a float64 image using mathematical operations.

    :param image: Input image (numpy array of dtype float64) with values from 0 to 1.
    :param alpha: Contrast control (>1.0 gives more contrast, <1.0 reduces contrast).
                  This function interprets alpha as the steepness of the contrast curve.
    :return: Contrast enhanced image (numpy array of dtype float64).
    """
    # Validate input
    if image.dtype != np.float64 or np.any(image < 0) or np.any(image > 1):
        raise ValueError(
            "Image must be of dtype float64 with values in the range [0, 1]"
        )

    # Implement a simple contrast enhancement curve that does not require a LUT
    # This curve is a basic power law transformation, similar in effect to adjusting LUT but suitable for float64 data
    enhanced_image = (image**alpha) / (image**alpha + (1 - image) ** alpha)

    return enhanced_image


def enhance_flow_contrast(flow, alpha=16, threshold=0.3):
    """
    Enhance the contrast of an optical flow field by adjusting the magnitudes of flow vectors
    using a smooth step function.

    :param flow: A NumPy array of shape (height, width, 2) containing the flow vectors.
    :param alpha: Contrast control parameter (>1.0 gives more contrast, <1.0 gives less contrast).
                  This parameter controls the steepness of the sigmoid-like function.
    :param threshold: Magnitude threshold as a fraction of the maximum magnitude in the flow field.
                      Magnitudes below this threshold are lowered, and magnitudes above are raised.
    :return: A NumPy array of the same shape as `flow`, with adjusted magnitudes.
    """
    # Calculate the magnitude and direction of the flow vectors
    magnitude = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
    direction = np.arctan2(flow[..., 1], flow[..., 0])

    # Normalize magnitudes to the range [0, 1]
    max_magnitude = np.max(magnitude)
    norm_magnitude = magnitude / max_magnitude

    # Apply smooth step function
    enhanced_norm_magnitude = 1 / (1 + np.exp(-alpha * (norm_magnitude - threshold)))

    # Scale back up to original max magnitude
    enhanced_magnitude = enhanced_norm_magnitude * max_magnitude

    # Reconstruct enhanced flow vectors
    enhanced_flow = np.zeros_like(flow)
    enhanced_flow[..., 0] = enhanced_magnitude * np.cos(direction)
    enhanced_flow[..., 1] = enhanced_magnitude * np.sin(direction)

    return enhanced_flow


def gaussian_blur_flow(flow, kernel_size=(5, 5), sigma=100):
    """
    Apply Gaussian blur to the optical flow field.

    :param flow: A NumPy array of shape (height, width, 2) containing the flow vectors.
    :param kernel_size: Size of the Gaussian kernel (tuple of integers).
    :param sigma: Standard deviation of the Gaussian kernel. If sigma is zero, it is computed as
                  `(kernel_size - 1) / 6`.
    :return: A NumPy array of the same shape as `flow`, with Gaussian blurred flow vectors.
    """
    # Split the flow vectors into x and y components
    flow_x = flow[..., 0]
    flow_y = flow[..., 1]

    # Apply Gaussian blur to x and y components separately
    flow_x_blurred = cv2.GaussianBlur(flow_x, kernel_size, sigma)
    flow_y_blurred = cv2.GaussianBlur(flow_y, kernel_size, sigma)

    # Reconstruct the blurred flow vectors
    blurred_flow = np.stack((flow_x_blurred, flow_y_blurred), axis=-1)

    return blurred_flow


def animate(SIZE_X, SIZE_Y, TIMESTEP, num_frames):

    images = [
        add_contrast(gen_grid(SIZE_X, SIZE_Y, time=i * TIMESTEP), alpha=20.0)
        for i in range(num_frames + 1)
    ]
    flows = []

    # Compute optical flow between consecutive frames
    for i in range(num_frames):
        flow = cv2.calcOpticalFlowFarneback(
            images[i],
            images[i + 1],
            None,
            pyr_scale=0.5,
            levels=10,
            winsize=128,
            iterations=3,
            poly_n=5,
            poly_sigma=1.2,
            flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
        )
        # add contrast
        # flow = gaussian_blur_flow(flow, sigma=100)
        # flow = enhance_flow_contrast(flow, threshold=0.6)

        flows.append(flow)

    fig, ax = plt.subplots(1, 2, figsize=(12, 6))

    def update(frame_num):
        """Update function for the animation."""
        # Clear previous content
        ax[0].clear()
        ax[1].clear()

        # Display the current frame
        ax[0].imshow(images[frame_num], cmap="gray")
        ax[0].set_title("Frame")

        # Display the flow magnitude as a heatmap
        magnitude = np.sqrt(
            flows[frame_num][..., 0] ** 2 + flows[frame_num][..., 1] ** 2
        )
        normalized_magnitude = cv2.normalize(magnitude, None, 0, 1, cv2.NORM_MINMAX)
        ax[1].imshow(normalized_magnitude, cmap="inferno")
        ax[1].set_title("Optical Flow Heatmap")

        # Hide axes
        for a in ax:
            a.axis("off")

    ani = FuncAnimation(
        fig,
        update,
        frames=num_frames,
        repeat=True,
        interval=64,
        repeat_delay=256,
    )

    plt.show()


def trim_border(array, size):
    """
    Trims or removes a specified border from an array.

    Parameters:
    - array: A 2D or 3D NumPy array from which the border will be removed.
             The first two dimensions are treated as spatial dimensions.
    - top: The size of the border to remove from the top.
    - bottom: The size of the border to remove from the bottom.
    - left: The size of the border to remove from the left.
    - right: The size of the border to remove from the right.

    Returns:
    - A new array with the specified border removed.
    """

    if array.ndim not in [2, 3]:
        raise ValueError("Array must be 2D or 3D.")

    # Calculate the new boundaries of the array
    new_top = size
    new_bottom = array.shape[0] - size
    new_left = size
    new_right = array.shape[1] - size

    # Trim the border
    trimmed_array = array[new_top:new_bottom, new_left:new_right]

    return trimmed_array


# Ultimate prediction function
def predict(image_a, image_b):

    # Add zero borders
    BORDER_WIDTH = 20
    image_a_ext = add_zero_border(image_a, BORDER_WIDTH)
    image_b_ext = add_zero_border(image_b, BORDER_WIDTH)

    # TODO: change resolution

    # Predict velocities
    flow = cv2.calcOpticalFlowFarneback(
        image_a_ext,
        image_b_ext,
        None,
        pyr_scale=0.5,
        levels=10,
        winsize=128,
        iterations=3,
        poly_n=5,
        poly_sigma=1.2,
        flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
    )

    # TODO: normalize velocities

    # Binarize image
    THRESHOLD = 0.5
    image_b_mask = np.where(image_b_ext >= THRESHOLD, 1, 0)

    # Mask velocities
    flow_masked = flow * np.expand_dims(image_b_mask, axis=-1)

    # Predict future grid
    prediction = predict_grid_modified(image_b, flow)

    # Trim border
    prediction_trimmed = trim_border(prediction, BORDER_WIDTH)

    # PLOTTING

    # Plotting
    fig, axs = plt.subplots(1, 5, figsize=(20, 4))

    # Display image_a
    axs[0].imshow(image_a, cmap="gray")
    axs[0].set_title("Image A")
    axs[0].axis("off")

    # Display image_b
    axs[1].imshow(image_b, cmap="gray")
    axs[1].set_title("Image B")
    axs[1].axis("off")

    # Display magnitude of predicted velocities
    magnitude = np.sqrt(flow_masked[..., 0] ** 2 + flow_masked[..., 1] ** 2)
    axs[2].imshow(magnitude, cmap="hot")
    axs[2].set_title("Predicted Velocities")
    axs[2].axis("off")

    # Display image_b mask
    axs[3].imshow(image_b_mask, cmap="gray")
    axs[3].set_title("Image B Mask")
    axs[3].axis("off")

    # Display predicted grid
    axs[4].imshow(prediction_trimmed, cmap="gray")
    axs[4].set_title("Predicted Grid")
    axs[4].axis("off")

    plt.show()

    return prediction_trimmed


if __name__ == "__main__":

    a = gen_grid(SIZE_X, SIZE_Y, time=0)
    b = gen_grid(SIZE_X, SIZE_Y, time=0 + TIMESTEP)
    predict(a, b)

    # # Process frames
    # frameA = add_contrast(a, alpha=2.0)
    # frameB = add_contrast(b, alpha=2.0)

    # flow = cv2.calcOpticalFlowFarneback(
    #     frameA,
    #     frameB,
    #     None,
    #     pyr_scale=0.1,
    #     levels=100,
    #     winsize=64,
    #     iterations=1000,
    #     poly_n=5,
    #     poly_sigma=1.2,
    #     flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
    # )

    # # flow = flow * 10e7
    # flow = scale_flow_vectors(flow, 0.001)

    # flow.astype("int16").tofile("./temp")

    # plt.figure("Frame A")
    # plt.imshow(frameA, cmap="gray")

    # plt.figure("Frame B")
    # plt.imshow(frameB, cmap="gray")

    # frameC = predict_grid_modified(frameB, flow)
    # plt.figure("Frame C")
    # plt.imshow(frameC, cmap="gray")

    # draw_flow_heatmap(flow)

    # plt.show()
    # SIZE_X, SIZE_Y, TIMESTEP, num_frames = 128, 128, 0.01, 248
    # animate(SIZE_X, SIZE_Y, TIMESTEP, num_frames)
