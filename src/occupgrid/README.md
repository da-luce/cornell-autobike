# LiDAR-Based Occupancy Grid Generation

This guide provides instructions for generating a live occupancy grid from our LiDAR data (Unitree L1) and creating an occupancy grid from recorded LiDAR data using ROS.

## Prerequisites
Make sure to have a ROS workspace set up and the necessary packages installed:
- `unitree_lidar_ros` (for handling LiDAR data)
- `pointcloud_to_grid` (for converting point clouds to an occupancy grid)

## Part 1: Creating a Live Occupancy Grid from LiDAR

### Steps

1. **Terminal 1** - Initialize the ROS core and build workspace:
    ```bash
    cd ~/catkin_ws
    catkin build pointcloud_to_grid
    source devel/setup.bash
    roscore
    ```

2. **Terminal 2** - Launch the LiDAR node:
    ```bash
    cd ~/catkin_ws/src/unilidar_sdk/unitree_lidar_ros
    catkin_make
    source devel/setup.bash
    roslaunch unitree_lidar_ros run.launch
    ```

3. **Terminal 3** - Launch the occupancy grid generation node:
    ```bash
    cd ~/catkin_ws
    catkin build pointcloud_to_grid
    source devel/setup.bash
    roslaunch pointcloud_to_grid demo.launch
    ```

4. **Terminal 4** - Launch RViz to visualize the occupancy grid:
    ```bash
    cd ~/catkin_ws
    catkin build pointcloud_to_grid
    source devel/setup.bash
    roslaunch pointcloud_to_grid rviz.launch
    ```

## Part 2: Generating Occupancy Grid from a Recorded Bag File

### Recording a Bag File

1. **Terminal 1** - Launch the LiDAR node to start recording:
    ```bash
    cd ~/catkin_ws/src/unilidar_sdk/unitree_lidar_ros
    catkin_make
    source devel/setup.bash
    roslaunch unitree_lidar_ros run.launch
    ```

2. **Terminal 2** - Start recording LiDAR data:
    ```bash
    rosbag record -O {enter_name_here}.bag /cloud
    ```
    - **Note**: Press `Ctrl+C` to stop recording when desired. Once recorded, close both terminals.

### Playing Back the Bag File and Generating Occupancy Grid

1. **Terminal 1** - Initialize the ROS core:
    ```bash
    cd ~/catkin_ws
    catkin build pointcloud_to_grid
    source devel/setup.bash
    roscore
    ```

2. **Terminal 2** - Play back the recorded bag file:
    ```bash
    cd ~/catkin_ws
    catkin build pointcloud_to_grid
    source devel/setup.bash
    rosbag play -l {enter_name_of_bag_file_here}.bag
    ```

3. **Terminal 3** - Launch the occupancy grid generation node:
    ```bash
    cd ~/catkin_ws
    catkin build pointcloud_to_grid
    source devel/setup.bash
    roslaunch pointcloud_to_grid demo.launch
    ```

4. **Terminal 4** - Launch RViz to visualize the occupancy grid:
    ```bash
    cd ~/catkin_ws
    catkin build pointcloud_to_grid
    source devel/setup.bash
    roslaunch pointcloud_to_grid rviz.launch
    ```

