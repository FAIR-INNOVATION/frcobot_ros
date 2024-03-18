# Gazebo ROS Model Color
[![Build Status](https://img.shields.io/jenkins/s/https/jenkins.qa.ubuntu.com/view/Precise/view/All%20Precise/job/precise-desktop-amd64_default.svg)](https://www.verlab.dcc.ufmg.br/gazebo_ros_model_color)
[![Version](https://img.shields.io/badge/version-1.0-brightgreen.svg)](https://www.verlab.dcc.ufmg.br/gazebo_ros_model_color)
[![License](https://img.shields.io/badge/license-GPL--3.0-blue.svg)](LICENSE)

This package contains a gazebo ros plugin which allows the user to modified dynamicly the color of the objects.

## Installation ##
> This setup was tested in ROS Kinetic, running on Ubuntu 16.04 LTS.

* [Install Gazebo](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
* Download from github and install the dependencies:

```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/verlab/gazebo_ros_model_color.git
$ cd ~/catkin_ws/
$ rosdep install --from-paths src/gazebo_ros_model_color --ignore-src -r -y
$ catkin_make
$ source devel/setup.bash
```


## Usage ##
This is an example how to use this plugin. Once imported it into a object, ROS should be able to provide a service to change the object color.

```xml
    <robot name="color_box">

  <link name="my_color_box">
    <inertial>
      <origin xyz="2 0 0" />
      <mass value="1.0" />
    <inertia  ixx="1.0" ixy="0.0"  ixz="0.0"  iyy="100.0"  iyz="0.0"  izz="1.0" />
    </inertial>
    <visual>
      <origin xyz="2 0 1"/>
      <geometry>
        <box size="1 1 2" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="2 0 1"/>
      <geometry>
        <box size="1 1 2" />
      </geometry>
    </collision>
  </link>

  <gazebo reference='my_color_box'>
    <visual>
      <plugin name="gazebo_ros_model_color" filename="libgazebo_ros_model_color.so">
        <robotNamespace>/</robotNamespace> <!-- Robot namespace -->
        <serviceName>/my_box_color</serviceName> <!-- Service topic name-->
        <color>1.0 1.0 1.0 1.0</color> <!-- Default Color -->
      </plugin>
    </visual>
  </gazebo>

  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/my_box</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```
To execute the example, type:
```sh
$ roslaunch gazebo_ros_model_color test.launch # this launch the gazebo world
```

Test:
```sh
$ rosservice call /gz_client/model_color "light_name: ''
diffuse: {r: 0.5, g: 0.5, b: 0.0, a: 1.0}
attenuation_constant: 0.0
attenuation_linear: 0.0
attenuation_quadratic: 0.0"
```


## References
- https://bitbucket.org/osrf/gazebo/src/aab36be8994a/plugins/?at=default
- http://answers.gazebosim.org/question/1377/visuals-material-update-from-modelplugin-in-ros-gazebo/
- How to create a gazebo ros plugin: http://gazebosim.org/tutorials?tut=ros_plugins


## Institution ##

Federal University of Minas Gerais (UFMG)  
Computer Science Department  
Belo Horizonte - Minas Gerais -Brazil 

## Laboratory ##

![VeRLab](https://www.dcc.ufmg.br/dcc/sites/default/files/public/verlab-logo.png)


**VeRLab:** Laboratory of Computer Vison and Robotics   
https://www.verlab.dcc.ufmg.br
