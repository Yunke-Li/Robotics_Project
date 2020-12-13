# Robotics Project
University Of Burgundy (VIBOT)
<br>
Supervisor: <br>
Raphael DUVERNE, Daniel BRAUN & Ralph SEULIN
<br>
Contributors: <br>
Yunke LI & Qi YANG
# Introduction

ROS (Robot Operating System) is a highly flexible software 
architecture for writing robot software programs. The theme of 
the robot project is to control the turtlebot3 to 
complete a series of specified operations under the 
ROS architecture. To complete this project, we use the 
construct online ROS learning and development platform.
In this platform, we use ubuntu as the main operating system.
 Use 3D physical simulation platform Gazebo to create a virtual 
 simulation environment. In addition, three-dimensional visualization 
 tool rviz is used to complete the visualization of existing data.
In this project, we have implemented a series of operations on 
the turtlebot3, such as control, create maps of environments,  
localize the robot in the environment, visualize data of the 
different Navigation processes, and make the robots perform 
path planning.



## Task Description 
* Create a script that moves the robot around with simple 
/cmd_vel publishing. See the range of movement of this new robot model. 
* Create the mapping launches, and map the whole environment. 
You have to finish with a clean map of the full cafeteria. 
Setup the launch to be able to localize the Turtlebot3 robot. 
* Set up the move base system so that you can publish a goal to 
move_base and Turtlebot3 can reach that goal without colliding 
with obstacles. 
* Create a program that allows the Turtlebot3 to navigate within 
the environment following a set of waypoints. Waypoints 
location are presented on the next page.







## Getting Started 

### Prerequisites 
```
ubuntu 16.04
ROS
```


### Usage example 

#### Task2: Map Creation
In order to perform autonomous Navigation, 
the robot must have a map of the environment. 
The robot will use this map for many things such as planning 
trajectories, avoiding obstacles, etc.
In our case, the goal is to create a map through the laser measurement 
from the robot before we start navigation in the following task.
<br>
The main process can be summarized as follows:

* Launch roscore
* Launch slam_gmapping node
```
roslaunch turtlebot_navigation_gazebo gmapping_demo.launch
```
* Visualize the simulation in graphic interface using rviz
```
rosrun rviz rviz
```
* Add the following displays 
    * Add LaserScan Display
        * Set the name of topic to /scan in LaserScan Properties
    * Fix the frame by Odom
    * Visualize the map 
        * Add map display
            * set the topic to /map in Map display properties
    * (Optional) Visualize the robot model
* Launch teleop node to control the robot manually to map the whole 
environment
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
* Save the map
```
rosrun map_server map_saver -f my_map
```
* Visualize the map headers
```
vi my_map.yaml
```
```
image: my_map.pgm
resolution: 0.050000
origin: [-18.600000, -17.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```
![](https://github.com/Yunke-Li/Figure_Repository/blob/main/RoboticProject/my_map.png?raw=true)

Till now, we have successfully create the map of the whole cafe which 
enables us to go to the next task.

#### Task3: Path Planning
Once we got a map, we can manage to navigate the robot which means to 
plan a a path, trajectory for the robot to follow to reach a specific 
goal while avoiding obstacles along the way.
<br>
The main process can be summarized as follows:
* Launch move base package
```
roslaunch husky_navigation move_base_demo.launch
```

* Visualize the simulation in graphic interface using rviz
```
rosrun rviz rviz
```
we can also check the costmap by setting topic to 
```
/move_base/global_costmap/costmap
```
or<br>
```
/move_base/local_costmap/costmap
```
in Map element
* Use the 2D Pose Estimate tool in order to provide an initial pose for the robot.
* Use the 2D Nav Goal tool in order to send a goal pose to the robot. Make sure to select
 an unoccupied (dark grey) or unexpected (light grey) location.



## Authors 

* **Yunke LI** - *Code and Implementation* - [Yunke LI](https://github.com/Yunke-Li)
* **Qi YANG** - *Code and Implementation* - [Qi YANG](https://github.com/yangqi7)



## License 

Please click [LICENSE.md](LICENSE.md) for more details.