# Navigation Stack (2022)
## Setup
### ROS Noetic, Ubuntu 20.04, Python 3.8

The current navigation stack which uses ROS's navigation stack including the move_base package
![move_base_node](https://wiki.ros.org/move_base?action=AttachFile&do=view&target=overview_tf.png) 

### Registry available ROS Package Requirements
```bash
sudo apt-get install ros-noetic-moveit-*
sudo apt-get install ros-noetic-gmapping-*
sudo apt-get install ros-noetic-turtlebot3-*
sudo apt-get install ros-noetic-explore-lite
```

### ROS Package Requirements
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/UBC-OpenRobotics/navigation.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws/
catkin build # or catkin_make
```

### Environment variables
```bash
export TURTLEBOT3_MODEL=waffle
```


## Navigation
**Troubleshooting**
```bash
Quaternion has length close to zero... discarding as navigation goal
```
This error is caused by an invalid quaternion pose. An example of an invalid pose could be if w = 0. Beware this is not the same as Euler coordinates.

## Mapping
We will be using the gmapping package to construct maps.

To save and use maps, we will be using the [map_server ros package](http://wiki.ros.org/map_server)

Saving a map 
Paramenters: map file name -f
```bash
rosrun map_server map_saver -f ~/map
```
Note: you should 

**Localization**

## Frontier Exploration
By using the boundary between open and unexplored space, frontier-based exploration is an approach to move a mobile robot to extend its map into new territory
until the entire environment has been explored.

**Current Status**
Under Development. No demo available.

**References:**
* Roswiki: [frontier_exploration package](http://wiki.ros.org/frontier_exploration)
* Implementation of this [Frontier Exploration](http://www.robotfrontier.com/papers/cira97.pdf) paper

**To Run:**
1. In run the bringup of the mobile robot base.
2. Run the frontier exploration node.
```bash
roslaunch navigation slam.launch
```

## Keyboard Navigation
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Navigation Servers

### Named Locations on Map:
Launch Server
```bash
rosrun navigation named_location.py
```

Save Current Location
```bash
rosservice call /navigate_named_location "action: 'Name' name: '<NAME>'"
```

Goto Saved Location
```bash
rosservice call /navigate_named_location "action: 'Goto' name: '<NAME>'"
```

### Relative Locations to Turtlebot:
Launch Server
```bash
rosrun navigation relative_location.py
```

Go to relative location. Location relative to move_base. x is axial movement (forward/back). y is lateral movement (side/side)
```bash
rosservice call /navigate_relative_location "x: <X-COORD> y: <Y-COORD>"
```

### Arm Adjustment Movement:
Imput vec is adjustment neccessary with respect to the coordinate frame of the base of the arm
```bash
rosrun navigation named_location
rosservice call /arm_adjustment_navigation "vec: {x: <X> y: <Y>, z: <Z>}"
```

### PID
See pid_follow.py to find the PID object that can be imported (python) for use.