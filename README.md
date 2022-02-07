Starship: Frontier Exploration for ROS2
---------------------------------------
This package implements frontier detection and environmental exploration for the Turtlebot3 using Cartographer, Navigation2 and OpenCV processing. It is designed to run on ROS2 Foxy.

### Turtlebot3 Operation
Installation:
1. Ensure all Turtlebot3 packages are installed, as well as Numpy, OpenCV and the latter's Python library.
2. Check out repository into ros_workspace/src
3. `cd ros_workspace && colcon build --packages-select starship --symlink-install`
4. `. install/local_setup.sh`

Running:
```
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 launch nav2_bringup navigation_launch.py
ros2 run starship explore
```

To visualize the frontier points in RViz2, add a visualization of the "/frontiers" MarkerArray topic. The points should appear immediately, but it will take several seconds for the robot to begin driving, as it waits for a transform between the map and base_link frames.

### Development
Setup:
1. Ensure all Turtlebot3 packages are installed, as well as Numpy, OpenCV and the latter's Python library.
2. Check out repository into ros_workspace/src
3. `cd ros_workspace && colcon build --packages-select starship --symlink-install`

Running:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 run starship explore
```