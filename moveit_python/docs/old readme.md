## moveit_python

This is a set of pure-python bindings to the ROS interface of MoveIt! based on the
earlier moveit_utils package that was developed as part of the chess_player package.

## Overview

There are three interfaces currently:

 * MoveGroupInterface -- used to move the arm using the move_group action.
 * PlanningSceneInterface -- used to add/remove collision and attached objects.
   Can also set the colors of objects.
 * PickPlaceInterface -- used to interface to the pick and place actions.

## MoveGroupInterface

The MoveGroupInterface is quite easy to use:

```python
import rospy
from moveit_python import *

rospy.init_node("moveit_py")
g = MoveGroupInterface("planning_group_name", "base_link")
```

Obviously, you might need different values for base_link, and your planning group
is probably not called "planning_group_name".

## PlanningSceneInterface

The PlanningSceneInterface allows you to easily insert and remove objects from
the MoveIt! planning scene. Starting with version 0.3.0, this module will try
to use the newer service-based approach to apply planning scene updates, as
this is much more robust than publishing messages over topics asynchronously.

```python
import rospy
from moveit_python import *

rospy.init_node("moveit_py")
# create a planning scene interface, provide name of root link
p = PlanningSceneInterface("base_link")

# add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
p.addCube("my_cube", 0.1, 1, 0, 0.5)

# do something

# remove the cube
p.removeCollisionObject("my_cube")
```

If for some reason you would prefer to not use the service, simply set
"use_service" to false in the various add/remove calls. Calling waitForSync
will republish messages as needed to synchronize the planning scene between
your script and MoveIt.

```python
p.addCube("my_cube", 0.1, 1, 0, 0.5, use_service=False)
p.addCube("my_other_cube", 0.1, 2, 0, 0.5, use_service=False)
p.waitForSync()
```

## PickPlaceInterface

```python
import rospy
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation

rospy.init_node("moveit_py")
# provide arm group and gripper group names
# also takes a third parameter "plan_only" which defaults to False
p = PickPlaceInterface("arm", "gripper")

g = Grasp()
# fill in g
# setup object named object_name using PlanningSceneInterface
p.pickup("object_name", [g, ], support_name = "supporting_surface")

l = PlaceLocation()
# fill in l
p.place("object_name" [l, ], goal_is_eef = True, support_name = "supporting_surface")
```

## Migration from moveit_utils

 * GraspingInterface renamed to PickPlaceInterface
   * The pick/place functions now return the entire action result, in moveit_utils they returned only the error_code. To access the error code as it used to be returned, you would use result.error_code.val
 * ObjectManager renamed to PlanningSceneInterface
   * remove function is now removeCollisionObject and removeAttachedObject
 * ArmInterface renamed to MoveGroupInterface


# MoveIt Pick and Place - Python version

The repository is a Python version of [this MoveIt tutorial](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html) on pick and place with slight changes. `moveit_msgs::Grasp` is not used.

<p align="center">
  <img src="./demo.gif" alt="gen3_lite rviz demo">
</p>

## Structure of package
```sh
.
├── CMakeLists.txt
├── package.xml
├── launch
│   ├── run_pick_place.launch
│   └── spawn_gen3_lite.launch
└── scripts
    └── main.py
```

## Installation
 - Package depends on [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) and was tested on `melodic-devel` version of it. 

 - Thus install `ros_kortex melodic` by following instructions in the repository.

 - Add this package to your `src` folder with:

```sh
git clone https://github.com/dabarov/moveit-pick-place-python.git
```

 - And build the workspace with:

```sh
catkin_make
```

## Usage

1. To start simulation run:
```sh
roslaunch pick_place_python spawn_gen3_lite.launch
```

2. To start pick and place control run:
```sh
roslaunch pick_place_python run_pick_place.launch
```
