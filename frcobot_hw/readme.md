# Quick start

1、First copy the frcobot_hw function package to the src folder under the ros user workspace, the general path is：~/catkin_ws/src

2、Compile the user space and execute the following command in the terminal：

cd ~/catkin_ws/

catkin_make

3、After the compilation is successful, you need to copy the files under lib to the /opt/ros/melodic/lib path. The general command is：

sudo cp ~/catkin_ws/src/frcobot_cpp_sdk_demo/lib/lib* /opt/ros/melodic/lib

4、The following command can be executed to run the robot command demo, which can switch the robot from manual mode to automatic mode：

rosrun frcobot_hw frcobot_cmd_demo

5、The following commands can be executed to run the robot status feedback node：

roslaunch frcobot_hw frcobot_hw.launch

6、You can execute the following commands to print and view the robot status feedback information：

rostopic echo /frcobot_status

7、Please refer to the "ROS C++ SDK User Manual" for the specific instructions of the robot SDK instructions.