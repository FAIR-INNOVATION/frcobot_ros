#include <pluginlib/class_loader.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#include <frcobot_gripper/gripper_io_pad.h>
#include "frcobot_gripper/FRRobot.h"
#include "frcobot_gripper/RobotError.h"
#include "frcobot_gripper/RobotTypes.h"
#include "xmlrpc-c/base.h"
#include "xmlrpc-c/client.h"

using namespace std;

int main(int argc, char** argv)
{
    pluginlib::ClassLoader<rviz_gripper_commander::GripperPanel> gripper_loader("rviz", "rviz::Panel");

    try {
        boost::shared_ptr<rviz_gripper_commander::GripperPanel> gripper_control = gripper_loader.createInstance("rviz_gripper_commander::GripperPanel");
        // boost::shared_ptr<rviz_gripper_commander::GripperAPI> gripper_control;
        gripper_control->openGripper();
        ROS_INFO("Gripper opened");
        sleep(5);
        gripper_control->closeGripper();
        ROS_INFO("Gripper closed");
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
    }

    return 0;
}