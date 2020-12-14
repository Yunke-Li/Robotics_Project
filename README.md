
# Robotics Project - Navigation on Turtlebot3

![](https://www.vibot.org/uploads/2/3/5/8/2358523/vibot3g_14.png)
<br>
**MSCV2, University Of Burgundy (VIBOT)**
<br>
**Supervisor: <br>**
**Raphael DUVERNE, Daniel BRAUN & Ralph SEULIN**
<br>
**Contributors: <br>**
**Yunke LI & Qi YANG**
## Index

- [Introduction](#Introduction)
- [Task Description](#Task-Description)
- [Task 1 Move the robot](#Task-1-Move-the-robot)
- [Task 2 Map Creation](#Task-2-Map-Creation)
- [Task 3 Path Planning](#Task-3-Path-Planning)
- [Task 4 Way points](#Task-4-Way-points)
- [References](#References)

## Introduction

ROS (Robot Operating System) is a highly flexible software 
architecture for writing robot software programs. The theme of 
the robot project is to control the turtlebot3 to 
complete a series of specified operations under the 
ROS architecture. To complete this project, we use the 
construct online ROS learning and development platform.
<br>
<br>
In this platform, we use ubuntu as the main operating system.
Use 3D physical simulation platform Gazebo to create a virtual 
simulation environment. In addition, 3D visualization 
tool rviz is used to complete the visualization of existing data.
<br>
<br>
In this project, we have implemented a series of operations on 
the turtlebot3, such as control, create maps of environments, 
localize the robot in the environment, visualize data of the 
different Navigation processes, and make the robots perform 
path planning.



## Task Description 
* Create a script that moves the robot around with simple 
/cmd_vel publishing. See the range of movement of this new robot model. 
<br><br>
* Create the mapping launches, and map the whole environment. 
You have to finish with a clean map of the full cafeteria. 
Setup the launch to be able to localize the Turtlebot3 robot. 
<br><br>
* Set up the move base system so that you can publish a goal to 
move_base and Turtlebot3 can reach that goal without colliding 
with obstacles. 
<br><br>
* Create a program that allows the Turtlebot3 to navigate within 
the environment following a set of waypoints. 


## Getting Started 

### Prerequisites 
```
ubuntu 16.04
ROS
```

### Installation
Put the /src folder in /catkin_ws like this:
```
/catkin_ws/src/...
```
**If you encounter problem launching packages, run:**
```
cd /catkin_ws
source devel/setup.bash
```
### Usage example 
#### Task 1 Move the robot
##### Analysis
Topic：*/cmd_vel* <br><br>
This topic is the one used to send velocity commands to the base of the robot. So, by sending a message through this topic, we can make the robot start moving and get the translation and rotation data.

Use commend: 
```
rostopic info /cmd_vel
```

get the type: 
```
geometry_msgs/Twist
```

So, this topic receives data of type **Twist**, and this Twist message comes from the package geometry_msgs.
##### Implementation
For task No.1, simply run 
```
python /src/task1_pkg/src/publisher.py
```
In this python script, we directly define and publish the Twist message 
in a fixed frequency to */cmd_vel* in order to control the robot linear 
and angular velocity.
```
linear_speed = 0.2
goal_distance = 10
angular_speed = 1.0
goal_angle = 0
```
Infos about Twist message
```
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
```

#### Task 2 Map Creation
##### Analysis
Node: **slam_gmapping**<br><br>
The node obtains data in the *sensor_msgs/Scan* message and to visualize the map obtained through topic or services. This node comes from the package Gmapping.

Subscribed Topics:
```
tf(tf/tfMessage):  Transformation related to the coordinate system
scan (secsor_msgs/Scan):  Laser scan data
```

published Topics:
```
map_metadata (nav_msgs/MapMetaData): Obtains map data from this periodically updated topic.
map (nav_msgs/OccupancyGrid): Gets the map from this periodically updated topic.
```
##### Implementation
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
        * **Set the name of topic to */scan* in LaserScan Properties**
    * Fix the frame by Odom
    * Visualize the map 
        * Add map display
            * set the topic to */map* in Map display properties
    * (Optional) Visualize the robot model
* Launch teleop node to control the robot manually to map the whole 
environment
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
![](https://github.com/Yunke-Li/Figure_Repository/blob/main/RoboticProject/task2.png?raw=true)
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

Till now, we have successfully created the map of the whole cafe which 
enables us to go to the next task.

#### Task 3 Path Planning
##### Analysis
Node: **amcl**<br><br>
The AMCL (Adaptive Monte Carlo Localization) package provides the amcl node, which uses the MCL system in order to track the localization of a robot moving in a 2D space.

Subscribed Topics:
```
scan (sensor_msgs/Scan):  Laser scanning.
tf (tf/tfMessage):  Coordinate conversion information.
map (nav_msgs/OccupancyGrid):  When the parameter use_map_topic is set, AMCL subscribes to the /map topic to obtain the map to complete laser-based positioning.
```
published Topics:
```
amcl_pose (geometry_msgs/PoseWithCovarianceStamped):  The estimated pose of the robot in the map, expressed in covariance.
particlecloud (geometry_msgs/PoseArray):  The pose estimation set maintained by the particle filter.
tf (tf/tfMessage): publish the conversion from mileage to map.
```
##### Implementation
Once we got a map, we can manage to navigate the robot which means to 
plan a a path, trajectory for the robot to follow to reach a specific 
goal while avoiding obstacles along the way.
<br>
The main process can be summarized as follows:
* Launch move base package
```
roslaunch husky_navigation move_base_demo.launch
```
![](https://github.com/Yunke-Li/Figure_Repository/blob/main/RoboticProject/task3.png?raw=true)
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
 an unoccupied or unexpected location.
 
#### Task 4 Way points
Now, our robot can successfully navigate to a certain target location. 
But if we want the robot to finally reach the target position after 
passing through several different coordinate points, we have to send goals
in order to accomplish the task.
<br>
Here we set 3 clients to publish goals to */move_base/goal*.The coordinates
are measured manually；
```
rostopic echo /move_base/goal
```
![](https://github.com/Yunke-Li/Figure_Repository/blob/main/RoboticProject/goals.png?raw=true)
##### Implementation
* Start rviz
```
rosrun rviz rviz
```
* Run
```
cd /catkin_ws/src
python sendgoal.py
```


 

## References
  - [WIKI ROS](http://wiki.ros.org/)
  - [The Construct Platform](http://theconstructsim.com)


## Authors 

* **Yunke LI** - *Code and Implementation* - [Yunke LI](https://github.com/Yunke-Li)
* **Qi YANG** - *Code and Implementation* - [Qi YANG](https://github.com/yangqi7)



## License 

Please click [LICENSE.md](LICENSE.md) for more details.