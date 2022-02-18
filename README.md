Starship: Frontier Exploration for ROS2
---------------------------------------
This package implements frontier detection and environmental exploration for the Turtlebot3 using Cartographer, Navigation2 and OpenCV processing. It is designed to run on ROS2 Foxy, and has been tested on a physical Turtlebot3 in its stock configuration.

Starship processes map data from SLAM with OpenCV, using Canny Edge Detection and then processing the results to quickly find the edge of explored space and target the center of each frontier line.


### Turtlebot3 Operation

#### Installation
1. Ensure all Turtlebot3 packages are installed, as well as Numpy, OpenCV and the latter's Python library.
2. Check out repository into ros_workspace/src
3. `cd ros_workspace && colcon build --packages-select starship --symlink-install`
4. `. install/local_setup.sh`

#### Running
```
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 launch nav2_bringup navigation_launch.py
ros2 run starship explore
```

To visualize the frontier points in RViz2, add a visualization of the "/frontiers" MarkerArray topic. The points should appear immediately, but it will take several seconds for the robot to begin driving, as it waits for a transform between the map and base_link frames.


### Development

#### Setup
1. Ensure all Turtlebot3 packages are installed, as well as Numpy, OpenCV and the latter's Python library.
2. Check out repository into ros_workspace/src
3. `cd ros_workspace && colcon build --packages-select starship --symlink-install`

#### Running
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 run starship explore
```


### Configuration

#### Subscribed Topics
* `/map` (nav_msgs/OccupancyGrid): The map produced by a SLAM node. This project was developed and tested with Google Cartographer, but it should be possible to use another SLAM tool. The topic name can be configured with the `map_topic` parameter.

#### Published Topics
* `/frontiers` (visualization_msgs/MarkerArray): Returns markers for RViz2, which display the frontier edges as well as the selected navigation targets.

#### Parameters
* `drive` (boolean): When set to false, navigation actions will not be sent to the Navigation server. This is useful for testing the frontier detection.

* `debug` (boolean): When true, OpenCV images will be displayed to test the map and edge processing.

* `map_topic` (string): The OccupancyGrid topic that map data will be supplied on. Defaults to `/map`.

* `base_frame` (string): The TF of the robot's base. Defaults to `base_link`.

* `map_frame` (string): The TF of the map. Defaults to `map`. Your SLAM system must supply a transform between the map frame and base frame.