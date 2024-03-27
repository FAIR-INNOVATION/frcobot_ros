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
