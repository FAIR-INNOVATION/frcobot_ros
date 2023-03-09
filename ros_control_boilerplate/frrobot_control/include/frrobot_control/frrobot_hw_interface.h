#ifndef FRROBOT_CONTROL__FRROBOT_HW_INTERFACE_H
#define FRROBOT_CONTROL__FRROBOT_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>

namespace frrobot_control
{

  /// @brief Hardware interface for a robot
  class FrRobotHWInterface : public ros_control_boilerplate::GenericHWInterface
  {
    public:
      /**
       * @brief Constructor
       * @param nh - Node handle for topics.
       */
      FrRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

      /** @brief Read the state from the robot hardware. */
      virtual void read(ros::Duration &elapsed_time);

      /** @brief Write the command to the robot hardware. */
      virtual void write(ros::Duration &elapsed_time);

      /** @brief Enforce limits for all values before writing */
      virtual void enforceLimits(ros::Duration &period);

  }; // class

} // namespace frrobot_control

#endif
