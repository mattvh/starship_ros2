Starship: Frontier Exploration for ROS2
---------------------------------------

### Development
Setup:
1. Check out repository into ros_workspace/src
2. `cd ros_workspace && colcon build --packages-select starship --symlink-install`

Running:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 run starship explore
```