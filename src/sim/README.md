# Bike Simulation

## Running the Sim

### (a) Start containers and run the sim

`docker compose up`

### (b) In a new terminal, run the following commands

1. `docker exec -it --user root autobike_dev bash`
2. `build`
3. `export WEBOTS_HOME=/usr/local/webots`
4. Double check if $WEBOTS_HOME was correctly exported: `echo $WEBOTS_HOME`
5. `ros2 launch sim robot_launch.py`

### (c) In a web browser, open the GUI

1. `http://localhost:8080/vnc.html`
2. Give [Webot](https://cyberbotics.com/) a minute

### Optional: Send commands to the driver via a new terminal

1. `docker exec -it --user root autobike_dev bash` for another terminal
v TODO: update robot driver to listen to output of pure pursuit container :D
2. `ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"`

### Optional: View in Rviz2

1. In a new terminal, open rviz: `rviz2`
2. Update frame (field to the right of `Fixed Frame`) to the name of the LiDAR as specifed in the world file (`our_lidar`)
3. View the pointcloud being published using the `add` button (look under `By topic`)

### Optional: View LiDAR in Webots

`View > Optional Rendering > Show Lidar Point Cloud`

## Tips

* Enable `Use Rosetta for x86/amd64 emulation on Apple Silicon` in `Settings > Features in Development` (the exact location seems to be different for everyone), otherwise it is horribly slow (still is extremely slow, since we are emulating a different architecture and also containerized--I anticipate the biggest hit is the emulation, and plan on running on the Jetson to see how much faster it is on native hardware).
* Select `View > Wireframe Rendering` to improve rendering performance.
* View LiDAR data under `View > Optional Rendering > Show Lidar Point Cloud`

## Resources

* Following [this](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html) tutorial! other versions of ROS WILL NOT WORK!!!
