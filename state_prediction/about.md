# State Prediction: Package Documentation

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
| $\beta$   | The yaw angle of the bike      | rad   |
| $\delta$  | The steering angle of the bike | rad   |

</center>


Effecitvely, bicycle states are represented as 1D `numpy` arrays of length 6:

$$[x, y, \dot{x}, \dot{y}, \Beta, \delta]$$

## State Simulation

### Dynamic Model 

The primary state prediction algorithim is an implentation of the dynamic model as defined in reference 1[^1].
The model relies on numerous physical constants of the bike, detailed below:

<center>

| Symbol    | Explanation                    | Units |
|-----------|--------------------------------|-------|
| $...$     | ...                            | ...   |

</center>

### Kinematic Model


## Performance

A "functional" approach to state simulation was chosen over an Object Oriented programming approach for numerous reasons:

- Python has limited and/or spotty support for object oriented paragims
- Representation of states as arrays lended to better readability and more succint code. i.e. `State.x, State.y, State.vel_x, etc.` vs. `x, y, vel_x, etc. = state`
- `numpy` provides many helpful, builtin functions for operating on arrays
- When used in conjuction with `numba`, the functional approach has significantly less memory overhead than the class appraoch, in addition to far superior performance

## References

[^1]: [J. Kong, M. Pfeiffer, G. Schildbach and F. Borrelli, "Kinematic and dynamic vehicle models for autonomous driving control design," 2015 IEEE Intelligent Vehicles Symposium (IV), Seoul, Korea (South), 2015, pp. 1094-1099, doi: 10.1109/IVS.2015.7225830.](https://ieeexplore.ieee.org/document/7225830)
- 
- 
- 
