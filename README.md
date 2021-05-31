# path_planning

A Path planning and visualization tool based on ROS.
Tested on Ubuntu 20.04 and ROS Noetic

Currently supported algorithms:
- RRT*

Dependencies
- OMPL
- Dynamic Reconfigure

A static tf publisher for simplicity
```
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 base_link map 1000
```
After starting the node:
- Use rqt to configure dynamic parameters
- Use the following service
```
rosservice call /input_params -- start_x start_y end_x end_y
```
Note that the type of the service parameters are 

Dynamic Reconfigure Parameters
- Number_of_Obstacles
- Radius_of_Obstacles
- Randomize_Obstacle_Radius
- RRTstar_Range
- Cost_Threshold
- Max_Solution_Time
