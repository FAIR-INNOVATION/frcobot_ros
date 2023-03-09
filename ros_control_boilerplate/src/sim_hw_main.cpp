/* 
   Desc:   Example ros_control main() entry point for controlling robots in ROS
*/

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <ros_control_boilerplate/sim_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<ros_control_boilerplate::SimHWInterface> sim_hw_interface
    (new ros_control_boilerplate::SimHWInterface(nh));
  sim_hw_interface->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, sim_hw_interface);
  control_loop.run(); // Blocks until shutdown signal recieved

  return 0;
}
