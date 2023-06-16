#include "frcobot_hw/frcobot_hw.h"
#include <frcobot_hw/status.h>

FrRobotStatusCtrl::FrRobotStatusCtrl()
{

    frrobot_status_ = nh_.advertise<frcobot_hw::status>("frcobot_status", 10);

    initTcp(); //Initialize the TCPIP connection with the robot
}

void FrRobotStatusCtrl::initTcp()
{
    nh_.getParam("robot_ip", ROBOTIP);
    nh_.getParam("robot_port", PORT);
    const char *robotIP = (char *)ROBOTIP.c_str();
    if (nh_.hasParam("robot_ip")) {
        ROS_INFO("%s", robotIP);
        
        ROS_INFO("%d", PORT);
    }
    //Set the server address and listening port through the struct sockaddr_in structure;
    memset(&serverSendAddr, 0, sizeof(serverSendAddr));
    serverSendAddr.sin_family = AF_INET;
    serverSendAddr.sin_addr.s_addr = inet_addr(robotIP);
    serverSendAddr.sin_port = htons(PORT);
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

    ROS_INFO("connected to status server");
}

void FrRobotStatusCtrl::read()
{
    recv_length = 0;
    // Receive data from the server, recv();
    recv_length = recv(confd, recv_buf, sizeof(recv_buf), 0);
    if (recv_length <= 0)
    {
        perror("recv");
    }
    else
    {
        frrobot_status.frame_count = recv_buf[2];
        frrobot_status.program_state = recv_buf[5];
        frrobot_status.error_code = recv_buf[6];
        frrobot_status.robot_mode = recv_buf[7];

        // joints
        for (int j = 0; j < 6; j++)
        {
            for (int i = 0; i < 8; i++)
            {
                doubleByte[i] = recv_buf[(j+1)*8+i];
            }
            memcpy(&doubleTemp, doubleByte, sizeof(double));
            frrobot_status.joints_sta[j] = doubleTemp;
        }

        // tcp
        for (int j = 0; j < 6; j++)
        {
            for (int i = 0; i < 8; i++)
            {
                doubleByte[i] = recv_buf[(j+7)*8+i];
            }
            memcpy(&doubleTemp, doubleByte, sizeof(double));
            frrobot_status.tcp_sta[j] = doubleTemp;
        }

        // torque
        for (int j = 0; j < 6; j++)
        {
            for (int i = 0; i < 8; i++)
            {
                doubleByte[i] = recv_buf[int(j+13.5)*8+i];
            }
            memcpy(&doubleTemp, doubleByte, sizeof(double));
            frrobot_status.torque_sta[j] = doubleTemp;
        }

        // tool_num
        for (int i = 0; i < 4; i++)
        {
            intByte[i] = recv_buf[13*8+i];
        }
        memcpy(&intTemp, intByte, sizeof(int));
        frrobot_status.tool_num = intTemp;

        // cl_dtg_ouput (DO8-DO15)
        for (int i = 0; i < 8; i++)
        {
            frrobot_status.cl_o_h[i] = (recv_buf[578] >> i) & 0x01;
        }

        // cl_dtg_ouput (DO0-DO7)
        for (int i = 0; i < 8; i++)
        {
            frrobot_status.cl_o_l[i] = (recv_buf[579] >> i) & 0x01;
        }

        // tl_dtg_ouput (end_DO1-end_DO0)
        for (int i = 0; i < 2; i++)
        {
            frrobot_status.tl_o_l[i] = (recv_buf[580] >> i) & 0x01;
        }

        // robot_motion_done
        for (int i = 0; i < 4; i++)
        {
            intByte[i] = recv_buf[int(72.625*8)+i];
        }
        memcpy(&intTemp, intByte, sizeof(int));
        frrobot_status.robot_motion_done = intTemp;

        // gripper_motion_done
        frrobot_status.gripper_motion_done = recv_buf[585];
    }
}

void FrRobotStatusCtrl::update()
{
    // update status tpoic
    frcobot_hw::status status_msg;

    status_msg.header.stamp = ros::Time::now();
    status_msg.frame_count = frrobot_status.frame_count;
    status_msg.program_state = frrobot_status.program_state;
    status_msg.error_code = frrobot_status.error_code;
    status_msg.robot_mode = frrobot_status.robot_mode;

    std::vector<double> joints_sta_(frrobot_status.joints_sta, frrobot_status.joints_sta+6);
    std::vector<double> tcp_sta_(frrobot_status.tcp_sta, frrobot_status.tcp_sta+6);
    std::vector<double> torque_sta_(frrobot_status.torque_sta, frrobot_status.torque_sta+6);
    status_msg.cur_joints_pose = joints_sta_;
    status_msg.cur_tcp_pose = tcp_sta_;
    status_msg.cur_joints_torque = torque_sta_;

    status_msg.tool_num = frrobot_status.tool_num;

    std::vector<uint8_t> cl_dgt_output_h_(frrobot_status.cl_o_h, frrobot_status.cl_o_h+8);
    std::vector<uint8_t> cl_dgt_output_l_(frrobot_status.cl_o_l, frrobot_status.cl_o_l+8);
    std::vector<uint8_t> tl_dgt_output_l_(frrobot_status.tl_o_l, frrobot_status.tl_o_l+2);
    status_msg.cl_dgt_output_h = cl_dgt_output_h_;
    status_msg.cl_dgt_output_l = cl_dgt_output_l_;
    status_msg.tl_dgt_output_l = tl_dgt_output_l_;

    status_msg.robot_motion_done = frrobot_status.robot_motion_done;
    status_msg.gripper_motion_done = frrobot_status.gripper_motion_done;
    
    frrobot_status_.publish(status_msg);
}

void FrRobotStatusCtrl::run()
{
    ros::Rate rate(125);
    while (ros::ok())
    {
        read();
        update();

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FrRobotStatusCtrl");

    FrRobotStatusCtrl FrRobotStatusCtrl;

    FrRobotStatusCtrl.run();
}