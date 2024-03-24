import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.grid_pred.grid_gen import gen_grid
from src.grid_pred.vec import (
    sigmoid,
    unify_flow_magnitudes,
    advect_grid_bilinear,
)


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


def advect_grid(image, flow):
    """
    Advect an image based on the provided flow field by looping over the flow field,
    then adding the original image's pixel values according to the flow vectors.

    Parameters:
    - image: A 2D numpy array representing the image.
    - flow: A 3D numpy array of shape (height, width, 2) representing the flow field.

    Returns:
    - A new 2D numpy array representing the advected image with accumulated values.
    """

    # Get the dimensions of the image
    height, width = image.shape

    # Create an output image initialized with zeros
    advected_image = np.zeros_like(
        image, dtype=np.float32
    )  # Use float32 to handle accumulations

    # Loop over each pixel in the flow field
    for y in range(height):
        for x in range(width):
            flow_vector = flow[y, x]
            # Calculate the source position for this flow vector
            from_y = int(round(y - flow_vector[1]))
            from_x = int(round(x - flow_vector[0]))

            # Check if the source position is within the bounds of the original image
            if 0 <= from_x < width and 0 <= from_y < height:
                # Add the value of the source pixel to the destination pixel
                advected_image[y, x] += image[from_y, from_x]

    # Convert the advected_image back to the original data type, e.g., uint8, clipping values to ensure they are valid
    advected_image = np.clip(advected_image, 0, 255).astype(image.dtype)

    return advected_image


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
    """
    Velocity grid prediction pipeline
    """

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
        winsize=image_a_ext.shape[0],  # Use edge length for prediction
        iterations=3,
        poly_n=5,
        poly_sigma=1.2,
        flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
    )

    # TODO: normalize velocities
    # flow = blur_dir(flow)
    flow = unify_flow_magnitudes(flow)
    # flow = np.multiply(flow, 5e8)
    flow = scale_flow_vectors(flow, 10)

    # Binarize image
    THRESHOLD = 0.3
    image_b_mask = sigmoid(image_b_ext, alpha=10, beta=0.4)

    # Mask velocities
    flow_masked = flow * np.expand_dims(image_b_mask, axis=-1)

    # Predict future grid
    prediction = advect_grid_bilinear(image_b_mask, flow_masked)

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
    axs[2].imshow(magnitude, cmap="magma")
    axs[2].set_title("Predicted Velocities")
    axs[2].axis("off")

    # Display image_b mask
    axs[3].imshow(image_b_mask, cmap="gray")
    axs[3].set_title("Image B Mask")
    axs[3].axis("off")

    # Display predicted grid
    axs[4].imshow(prediction, cmap="gray")
    axs[4].set_title("Predicted Grid Trimmed")
    axs[4].axis("off")

    plt.show()

    return prediction_trimmed


if __name__ == "__main__":

    # Occupancy grid
    SIZE_X = 128
    SIZE_Y = 128
    TIMESTEP = 0.5

    a = gen_grid(SIZE_X, SIZE_Y, time=0)
    b = gen_grid(SIZE_X, SIZE_Y, time=0 + TIMESTEP)
    predict(a, b)
