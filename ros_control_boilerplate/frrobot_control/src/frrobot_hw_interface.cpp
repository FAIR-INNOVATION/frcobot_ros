/*
   Desc:   Example ros_control hardware interface blank template for the FrRobot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <frrobot_control/frrobot_hw_interface.h>
#include "frrobot_control/udp_node.h"
#include "frrobot_control/tool.h"
#include <math.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <frrobot_control/FRRobot.h>
#include <frrobot_control/RobotError.h>
#include <frrobot_control/RobotTypes.h>

#define MAXLINE 4096
#define PORT_CMD 8080
#define SERVERIP "192.168.58.2"

int confd;
int len;
int flag = 1;
float joints[6];
float a = 0.0;
float v = 0.0;
float t = 0.002;
float lat = 0.002;
float gain = 0.0;
float joints_sta[6] = {0, 0, 0, 0, 0, 0}; // 关节数组
socklen_t sendaddr_length;
char recvLine[MAXLINE];
char sendCmdLine[MAXLINE];
char sendStaLine[MAXLINE];
char recv_buf[MAXLINE];
char send_buf[MAXLINE];
struct sockaddr_in serverSendAddr;

namespace frrobot_control
{

  FrRobotHWInterface::FrRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    // Set the server address and listening port through the struct sockaddr_in structure;
    memset(&serverSendAddr, 0, sizeof(serverSendAddr));
    serverSendAddr.sin_family = AF_INET;
    serverSendAddr.sin_addr.s_addr = inet_addr(SERVERIP);
    serverSendAddr.sin_port = htons(PORT_CMD);
    sendaddr_length = sizeof(serverSendAddr);

    // Use socket() to generate a socket file descriptor;
    if ((confd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      ROS_INFO("socket() error");
      perror("socket() error");
      exit(1);
    }

    if (connect(confd, (struct sockaddr *)&serverSendAddr, sizeof(serverSendAddr)) < 0)
    {
      ROS_INFO("connect() error");
      perror("connect() error");
      exit(1);
    }

    ROS_INFO("connected to server");

    ROS_INFO_NAMED("frrobot_hw_interface", "FrRobotHWInterface Ready.");
  }

  void FrRobotHWInterface::write(ros::Duration &elapsed_time)
  {
    // ROS_INFO("write");
    // Safety
    enforceLimits(elapsed_time);

    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      joints[joint_id] = joint_position_command_[joint_id] * 180 / M_PI;
    }
    // printf("joints: %f,%f,%f,%f,%f,%f;\n", joints[0],joints[1],joints[2],joints[3],joints[4],joints[5]);
    sprintf(send_buf, "ServoJ(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], a, v, t, lat, gain);

    len = strlen(send_buf);
    sprintf(sendCmdLine, "/f/bIII123III376III%dIII%sIII/b/f", len, send_buf);
    // Send data to the server, send();
    int send_length = 0;
    send_length = send(confd, sendCmdLine, sizeof(sendCmdLine), 0);
    if (send_length < 0)
    {
      perror("send() error");
    }
    else
    {
      // printf("write_sendmsg: %s;\n", sendCmdLine);
    }
    int recv_length = 0;
    // Receive data from the server, recv();
    recvLine[MAXLINE] = '\0';
    recv_length = recv(confd, recvLine, sizeof(recvLine), 0);
    if (recv_length <= 0)
    {
      perror("recv");
    }
    else
    {
      // printf("write_recvmsg: %s;\n", recvLine);
    }
  }

  void FrRobotHWInterface::read(ros::Duration &elapsed_time)
  {
    int send_length_sta = 0;
    sprintf(sendStaLine, "/f/bIII123III375III25IIIGetActualJointPosRadian()III/b/f");
    send_length_sta = send(confd, sendStaLine, sizeof(sendStaLine), 0);
    if (send_length_sta < 0)
    {
      perror("sendto() error");
      exit(1);
    }
    else
    {
      // printf("read_sendmsg: %s;\n", sendStaLine);
    }

    // ROS_INFO("read");

    int recv_length = 0;
    recvLine[MAXLINE] = '\0';
    // 接收服务器的数据，recvfrom()；
    recv_length = recv(confd, recvLine, sizeof(recvLine), 0);
    if (recv_length <= 0)
    {
      perror("recv");
    }
    else
    {
      // printf("read_recvmsg: %s;\n", recvLine);
      int pos = 0;
      int jointsDataLen = 0;
      std::string tempJoints = recvLine;
      for (int i = 0; i < 3; i++)
      {
        pos = tempJoints.find("III") + 3;
        tempJoints = tempJoints.substr(pos);
      }
      pos = tempJoints.find("III");
      jointsDataLen = stoi(tempJoints.substr(0, pos));
      tempJoints = tempJoints.substr(pos + 3, jointsDataLen);
      for (int i = 0; i < 6; i++)
      {
        pos = tempJoints.find(",");
        joints_sta[i] = stof(tempJoints.substr(0, pos));
        tempJoints = tempJoints.substr(pos + 1);
      }
      for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
      {
        joint_position_[joint_id] = joints_sta[joint_id];
        // joint_position_[joint_id] = joint_position_command_[joint_id];
      }
      // printf("recvmsg: %f,%f,%f,%f,%f,%f", joints_sta[0],joints_sta[1],joints_sta[2],joints_sta[3],joints_sta[4],joints_sta[5]);
    }
  }

  void FrRobotHWInterface::enforceLimits(ros::Duration &period)
  {
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    // vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
  }

} // namespace frrobot_control
