# Bike Simulation

## Running the Sim

Start containers and run the sim:

1. `docker compose up`
2. `docker exec -it --user root autobike_dev bash`
3. `build`
4. `ros2 launch sim robot_launch.py`
5. Install Webots with `y` if necessary

Open the GUI

5. `http://localhost:8080/vnc.html`
6. Give [Webot](https://cyberbotics.com/) a minute

Send commands to the driver

1. `docker exec -it --user root autobike_dev bash` for another terminal
v TODO: update robot driver to listen to output of pure pursuit container :D
1. `ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"`

View in Rviz2

* Update frame to name of LiDAR (`our_lidar`)
* View the pointcloud being published

## Tips

Enable `Use Rosetta for x86/amd64 emulation on Apple Silicon` in `Settings > Features in Development`, otherwise it is horribly slow (still is extremely slow, since we are emulating a different architecture and also containerized--I anticipate the biggest hit is the emulation, and plan on running on the Jetson to see how much faster it is on native hardware)

Select `View > Wireframe Rendering` to improve performance.

View LiDAR data under `View > Optional Rendering > Show Lidar Point Cloud`

## Resources

* Following [this](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html) tutorial! other versions of ROS WILL NOT WORK!!!
