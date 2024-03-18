# frcobot_ros

#### Install a virtual box in your PC (optional)

I will provide some recoureces for windows users. If you are using Linux then you can skip some guides.

See the instruction from another resources to install virtualbox with Ubuntu 20.04

https://www.virtualbox.org/wiki/Downloads

If the problem appeard with permission for user:

open the terminal:

```bash
su root
nano /etc/sudoers
```
Write in there this codes
```bash
vboxuser ALL=(ALL:ALL) ALL
%vboxuser ALL=(ALL) ALL
```

![alt text](./root.png)

Save the file and write "exit"

#### Install ROS

```bash
sudo apt update
```
```bash
sudo apt install git python3-pip python3-schedule -y
```
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/TPODAvia/ROS1-installation.git
chmod +x ROS1-installation/ROS.sh
sudo ./ROS1-installation/ROS.sh
```
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/noetic/setup.bash
sudo apt-get install python3-rosdep -y
```

```bash
sudo cp ~/catkin_ws/src/frcobot_ros/frcobot_hw/lib/* /opt/ros/noetic/lib

sudo apt-get install -y ros-noetic-rosparam-shortcuts ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-moveit -y

<!-- sudo apt-get install ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers ros-noetic-velocity-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control -->


cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
sudo rosdep init

rosdep update
rosdep install --from-paths src --ignore-src -y
catkin make
```