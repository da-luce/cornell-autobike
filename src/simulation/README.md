## Step-by-Step Checklist for Gazebo Simulation

1. **Create Simulation Package**: Create a new ROS package called `bike_simulation` using `catkin_create_pkg`.
2. **Add URDF/SDF Model**: Create or import the bike’s URDF or SDF model and place it in `models/`.
3. **Set Up Gazebo World**: Add a custom or default Gazebo world file and store it in `worlds/`.
4. **Write Launch File**: Create a Gazebo launch file that loads the bike model and world, and put it in `launch/`.
5. **Configure ROS Control**: Set up control configurations (YAML files) for the bike in `config/`.
6. **Integrate ros_control in URDF**: Add transmission elements in the URDF to enable `ros_control`.
7. **Build Workspace**: Run `catkin_make` and source your workspace.
8. **Test Simulation**: Launch the simulation with `roslaunch bike_simulation <your_launch_file>.launch`.
9. **Publish Commands**: Use ROS nodes or CLI to send commands to control the bike’s joints in the simulation.
