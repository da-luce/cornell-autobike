# State Prediction: Package Documentation

## Usage

### Finding possible states

Given the current state of the bike `state = [x,y,ẋ,ẏ,ψ,δ]`, state matrix
differentials `diff = [∂x,∂y,∂ẋ,∂ẏ,∂ψ,∂δ]`, and input resolutions
`res = [Δs, Δt]`, determine the possible states with

```python
get_possible_states(state, diff, res)
```

This will return an matrix of all obtainable state arrays within the next
timestep (currently defined in `constants.py`). Alteratively, to obtain the
indices of these states within the state matrix, use

```python
get_possible_indices(state, diff, res)
```

This will return a matrix of the indices of the obtainable states within the
state matrix.

**Example:**

```python
# Example state
state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])

# Example differentials
differentials = np.array([0.001, 0.001, 0.001, 0.001,
                          np.radians(0.0001), np.radians(0.0001)])

# Example input resolutions
res = optimize_input_res()

# Get array of all possible states achievable in DT from current state
possible_states = get_possible_states(state, differentials, res)
```

> ℹ️ Continue reading to learn about state, differentials, and input resolutions

## State Representation

<p align="center">
  <img src="https://miro.medium.com/v2/resize:fit:720/format:webp/1*A5wYkyE1d_6tg_BUrpMOeg.png" />
</p>

<center>

| Symbol    | Explanation                    | Units |
|-----------|--------------------------------|-------|
| $x$       | The x position of the bike     | m     |
| $y$       | The y position of the bike     | m     |
| $\dot{x}$ | The x velocity of the bike     | m/s   |
| $\dot{y}$ | The y velocity of the bike     | m/s   |
| $\psi$    | The yaw angle of the bike      | rad   |
| $\delta$  | The steering angle of the bike | rad   |

</center>

Effectively, bicycle states are represented as 1D `numpy` arrays of length 6:

$$[x, y, \dot{x}, \dot{y}, \psi, \delta]$$

## State Simulation

### Dynamic Model

The primary state prediction algorithm is an implementation of the dynamic model
as defined in Kong et al. (2015)[^1].

The model relies on a variety physical constants of the bike, detailed below:

<center>

| Symbol    | Explanation                    | Units |
|-----------|--------------------------------|-------|
| $...$     | ...                            | ...   |

</center>

### Kinematic Model

A kinematic would be less accurate than a dynamic model, as it ignores the
actual forces acting upon the bike. However, it would also be computationally
less expensive. More investigation required.  

## Terminology

### Input

An input is any parameter that we control on the bike. Right now we have two
inputs: the steering angle (turning the front wheel) and the acceleration (the
motor on the rear of the wheel)

### Input Delta

At each timestep, we can change the values of certain inputs: this is the *input
delta*, e.g. how much we turn the front wheel at this timestep  

### Input Resolution

At each timestep, we test a range of discrete values for each *input*: the
common difference between all these values is the *input resolution*.
E.g. if we have an *input resolution* of 1&deg; for the steering angle, we will
test a range of angles varying by 1&deg; each between the
minimum and maximum angle that we can turn the wheel at each timestep. (for more
information, see [Optimization](#optimization))

## Performance

A "functional" approach to state simulation was chosen over an Object Oriented
programming approach for numerous reasons:

- Python has limited and/or spotty support for object oriented paradigms
- Representation of states as arrays lend to better readability and more
  succinct code. i.e. `State.x, State.y, State.vel_x, etc.` vs.
  `x, y, vel_x, etc. = state`
- `numpy` provides many helpful, builtin functions for operating on arrays
- When used in conjunction with `numba`, the functional approach has
  significantly less memory overhead than the class approach, in addition to far
  superior performance

## Optimization

As previously discussed, `get_possible_states()` works by testing a set of
values between $p_{min}$ and $p_{max}$ for each input parameter $p$.

In order to achieve this, we choose some $\Delta p$ and test the range of values
$\{p : p = p_{min} + \Delta p \cdot n, n \epsilon \mathbb{W}, p < p_{max}\}$
using numpy's `numpy.arange` function.

As $\Delta p$ decreases, `get_possible_states()` calculates more possible
states, which are then rounded to fit within the state matrix. If $\Delta p$  is
too small, we waste computation power on computing multiple states that are
ultimately rounded to the same state. Likewise, if $\Delta p$ is too large, we
fail to return all possible states within the state matrix. For succinctness, we
will define the set of all input resolutions as:

$$
X = \left\{
    \begin{array}{l}
        \Delta p_1 \\
        \Delta p_2 \\
        \vdots     \\
        \Delta p_n \\
    \end{array}
\right\}
$$

Where $n$ is the total number of inputs. For any given input, we can define our
choice in $\Delta p$ as an attempt minimize the time complexity of the
`get_possible_states()` alogrithim,given by $O(X)$, while maximizing the number
of calculated state $N(X)$. Note that the time complexity can be calculated as

$$
O(X)= \prod_{i = 1}^{n}
\lfloor
  \frac {p_{i_{max}}-p_{i_{min}}}{\Delta p_i}
\rfloor
$$

for a model with $n$ input parameters, given that the possible input
combinations are calculated through nested for loops. As shown, as $\Delta p_i$
decreases, the time complexity increases by a factor of $\frac {1}{\Delta p_i}$.
In contrast, the value of $N(X)$ can only be roughly estimated by running
`get_possible_states()` on an arbitrary state and input resolutions.

> Note: is there a better method to calculating $N(X)$?

We are thus dealing with a *multi-objective optimization problem*[^2] (on multi
variable functions too!).

> [pymoo](https://pymoo.org/getting_started/preface.html) provides some helpful
> information on this type of problem

We can define our problem as

$$
\begin{align}
\begin{split}
\quad& \min \quad \quad \quad \quad O(X), N(X) \\
\quad& \text{s.t.} \quad \quad \quad \quad \Delta p_{i}^{L} < \Delta p_i \leq \Delta p_{i}^{U}  \quad i = 1,..,n \\
\end{split}
\end{align}
$$

Where $\Delta p_i$ represents the $i$-th input resolution to be optimized, and
$\Delta p_{i}^{L}$ and $\Delta p_{i}^{U}$ are its lower and upper bounds
respectively.

We will defined the lower bound $\Delta p_{i}^{L}$ as $0$ for all inputs and
$\Delta p_{i}^{U}$ as a corresponding element in the differentials. For
instance, if the differentials defines that states only vary by $1^{\circ}$,
then $\Delta p_{i}^{U}$ for the steering angle will defined as $1^{\circ}$.

> Note: is there a way to prove that any $\Delta p_{i}^{U}$ greater than the
> resolution defined in the differentials is too large? (So far it simply
> appears this way)

## References

[^1]: [J. Kong, M. Pfeiffer, G. Schildbach and F. Borrelli, "Kinematic and dynamic vehicle models for autonomous driving control design," 2015 IEEE Intelligent Vehicles Symposium (IV), Seoul, Korea (South), 2015, pp. 1094-1099, doi: 10.1109/IVS.2015.7225830.](https://ieeexplore.ieee.org/document/7225830)
[^2]: Wikipedia contributors. (2023, March 28). Multi-objective optimization. In Wikipedia, The Free Encyclopedia. Retrieved 20:53, March 30, 2023, from https://en.wikipedia.org/w/index.php?title=Multi-objective_optimization&oldid=1147004915
