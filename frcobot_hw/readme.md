20230301简要说明：
1、首先，将frcobot_hw功能包拷贝到ros用户工作空间下的src文件夹，一般路径为：~/catkin_ws/src

2、其次，编译用户空间，在终端中执行如下指令：

cd ~/catkin_ws/
catkin_make

3、编译成功后，需要将lib下面的文件拷贝到/opt/ros/melodic/lib路径下，一般指令为：

sudo cp ~/catkin_ws/src/frcobot_cpp_sdk_demo/lib/lib* /opt/ros/melodic/lib

4、可执行如下指令运行机器人指令demo，可使机器人从手动模式切换为自动模式：

rosrun frcobot_hw frcobot_cmd_demo

5、可执行如下指令运行机器人状态反馈节点：

roslaunch frcobot_hw frcobot_hw.launch

6、可执行如下指令打印查看机器人状态反馈信息：

rostopic echo /frcobot_status

7、机器人SDK指令的具体使用说明请查看《ROS C++ SDK用户手册》