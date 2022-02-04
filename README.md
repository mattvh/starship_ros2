Starship: Frontier Exploration for ROS2
---------------------------------------

### Development
Setup:
1. Ensure all Turtlebot3 packages are installed.
2. Check out repository into ros_workspace/src
3. `cd ros_workspace && colcon build --packages-select starship --symlink-install`

Running:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 run starship explore
```

To visualize the frontier points in RViz2, add a visualization of the "/frontiers" MarkerArray topic.