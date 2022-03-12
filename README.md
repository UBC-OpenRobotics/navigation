# Navigation Stack (2022)
### ROS Noetic, Ubuntu 20.04, Python 3.8

The current navigation stack which uses ROS's navigation stack including the move_base package
![move_base_node](https://wiki.ros.org/move_base?action=AttachFile&do=view&target=overview_tf.png) 

## Gmapping and Saving the map

Saving the map
```bash
$ rosrun map_server map_saver -f ~/map
```


## Frontier Exploration
By using the boundary between open and unexplored space, frontier-based exploration is an approach to move a mobile robot to extend its map into new territory
until the entire environment has been explored.

**References:**
* Roswiki: [frontier_exploration package](http://wiki.ros.org/frontier_exploration)
* Implementation of this [Frontier Exploration](http://www.robotfrontier.com/papers/cira97.pdf) paper

**To Run:**
1. In run the bringup of the mobile robot base.
2. Run the frontier exploration node.
```bash
$ roslaunch navigation slam.launch
```

## Keyboard Navigation
```bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```