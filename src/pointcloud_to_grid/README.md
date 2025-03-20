# PointCloud to Grid

## Overview
This package converts `sensor_msgs/PointCloud2` LIDAR data into `nav_msgs/OccupancyGrid` 2D map data based on intensity and/or height. It is designed for use in ROS 2 and provides a real-time occupancy grid for autonomous navigation.

## Dependencies
- **ROS 2** (Humble or later recommended)
- **Point Cloud Library (PCL)**

## Usage
Launch the node with:
```sh
ros2 launch pointcloud_to_grid demo.launch.py
```

## Changing the PointCloud Topic
The default topic is currently set to `/pointcloud`. To change this, modify the following files:

- `launch/demo.launch.py`
- `launch/grid_trajectory.launch.py`

Locate any occurrences of `"/pointcloud"` and replace them with the desired topic name.


## Configuring Simulation Parameters
To adjust parameters for the simulation, modify `launch/robot_launch.py` accordingly:

```python
pointcloud_to_grid_node = Node(
    package='pointcloud_to_grid',
    executable='pointcloud_to_grid_node',
    output='screen',
    parameters=[ # List of parameters you can tweak
        {'cloud_in_topic': '/pointcloud'},  # Set your topic here
        {'position_x': 0.0}, 
        {'position_y': 0.0},
        {'verbose1': False},
        {'verbose2': False},
        {'cell_size': 0.0},
        {'length_x': 100.0},
        {'length_y': 100.0},
        {'mapi_topic_name': 'intensity_grid'},
        {'maph_topic_name': 'height_grid'},
    ]
)
```

## Nodes
### `pointcloud_to_grid_node`
**Subscribed Topics:**
- `/pointcloud` (`sensor_msgs/PointCloud2`)

**Published Topics:**
- `/occupancy_grid` (`nav_msgs/OccupancyGrid`)
