import numpy as np
import pytest

import src.state_pred.constants as cst
from src.state_pred.bike_sim import (
    get_possible_indices,
    get_possible_states,
    optimize_input_res,
    round_state,
    round_to_multiple,
)


@pytest.fixture
def sample_state():
    """Fixture providing a sample state for testing."""
    return np.array([0, 0, 1, -2, np.radians(-10), np.radians(-20)])


@pytest.fixture
def differentials():
    """Fixture providing differentials for testing."""
    return np.array([0.001, 0.001, 0.001, 0.001, np.radians(0.001), np.radians(0.001)])


@pytest.fixture
def input_res():
    """Fixture providing input resolutions for testing."""
    return optimize_input_res()


def test_get_possible_states(sample_state, differentials, input_res):
    """Test get_possible_states function to ensure it returns expected number of states."""
    possible_states = get_possible_states(sample_state, differentials, input_res)

    # Validate the shape of the returned array
    assert possible_states.shape[1] == 6, "Each possible state should have 6 elements"

    # Ensure the function returns multiple possible states
    assert (
        possible_states.shape[0] > 0
    ), "Function should return multiple possible states"

    # Check that the returned states are within valid ranges (indirectly tests `valid_state`)
    for state in possible_states:
        assert (
            -cst.STEER_ANGLE_MAX <= state[5] <= cst.STEER_ANGLE_MAX
        ), "Steering angle is out of valid range"

        speed = np.sqrt(state[2] ** 2 + state[3] ** 2)
        assert cst.SPEED_MIN <= speed <= cst.SPEED_MAX, "Speed is out of valid range"


def test_get_possible_indices(sample_state, differentials, input_res):
    """Test get_possible_indices function to ensure correct indices are returned."""
    indices = get_possible_indices(sample_state, differentials, input_res)

    # Ensure the returned indices are valid
    assert indices.shape[1] == 6, "Each index set should have 6 elements"
    assert indices.shape[0] > 0, "Function should return multiple indices"

    # Check that indices are non-negative and consistent with state matrix indexing
    for idx in indices:
        assert np.all(idx >= 0), "Indices should be non-negative"
        assert np.all(np.isfinite(idx)), "Indices should be finite numbers"


def test_optimize_input_res():
    """Test optimize_input_res function to ensure it returns expected resolution values."""
    res = optimize_input_res()

    # Validate the length of the resolution array
    assert (
        len(res) == 2
    ), "Resolution array should have two elements (throttle and steering)"

    # Check that the resolutions are positive
    assert np.all(res > 0), "Resolutions should be positive"


def test_round_state(sample_state, differentials):
    """Indirectly test round_state function through get_possible_states."""
    rounded_state = round_state(sample_state, differentials)

    # Ensure the state is correctly rounded to the specified differentials
    for i, (value, diff) in enumerate(zip(rounded_state, differentials)):
        assert value == pytest.approx(
            round(sample_state[i] / diff) * diff
        ), f"State element {i} was not rounded correctly"


def test_round_to_multiple():
    """Test round_to_multiple function directly."""
    assert round_to_multiple(5.5, 10) == 10, "5.5 should round to 10 with multiple 10"
    assert round_to_multiple(6.3, 15) == 0, "6.3 should round to 0 with multiple 15"
    assert round_to_multiple(-4.5, 2) == -4, "-4.5 should round to -4 with multiple 2"
