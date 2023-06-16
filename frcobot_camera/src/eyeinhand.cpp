#include "frcobot_camera/frcobot_rvs.h"

FrCobotRVSCon::FrCobotRVSCon() {
    nh.getParam("rvs_ip", RVS_IP);
    nh.getParam("rvs_port", RVS_PORT);
    rvsIP = (char *)RVS_IP.c_str();
    if (nh.hasParam("rvs_ip")) {
        ROS_INFO("RVS IP:%s", rvsIP);
        ROS_INFO("RVS Port:%d", RVS_PORT);
    }
    else
    {
        ROS_ERROR("The 'rvs_ip' parameter cannot be found in ROS parameter server.");
    }

    if (nh.hasParam("robot_ip")) {
        nh.getParam("robot_ip", ROBOTIP);
        robotIP = (char *)ROBOTIP.c_str();
        ROS_INFO("FRcobot IP:%s", robotIP);
        robot.RPC(robotIP);     //Establish a communication connection with the robot controller
    } 
    else
    {
         ROS_ERROR("The 'robot_ip' parameter cannot be found in ROS parameter server.");
    }
    
}

void FrCobotRVSCon::rvsconnect()
{
    //Set the server address and listening port through the struct sockaddr_in structure;
    memset(&rvsserverSendAddr, 0, sizeof(rvsserverSendAddr));
    rvsserverSendAddr.sin_family = AF_INET;
    rvsserverSendAddr.sin_addr.s_addr = inet_addr(rvsIP);
    rvsserverSendAddr.sin_port = htons(RVS_PORT);
    rvs_sendaddr_length = sizeof(rvsserverSendAddr);

    // Use socket() to generate a socket file descriptor;
    if ((rvs_confd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_ERROR("socket() error");
        exit(1);
    }

    if (connect(rvs_confd, (struct sockaddr *)&rvsserverSendAddr, sizeof(rvsserverSendAddr)) < 0)
    {
        ROS_ERROR("connect() error");
        exit(1);
    }

    ROS_INFO("Connected to RVS Server");
}

void FrCobotRVSCon::rvsdisconnect() {
    int ret = close(rvs_confd);
    if (ret == 0)
    {
        ROS_INFO("Disconnected to RVS Server");
    } else {
        ROS_ERROR("Failed to disconnected, error:%d", ret);
    }
}

void FrCobotRVSCon::writervs(const char* sendBuf) {
    int send_length = 0;
    // Send data to the server, send();
    send_length = send(rvs_confd, sendBuf, strlen(sendBuf)+1, 0);
    if (send_length < 0)
    {
      ROS_ERROR("Send error");
    }
    else
    {
      ROS_INFO("Sendmsg: %s", sendBuf);
    }
}

void FrCobotRVSCon::readrvs(char *recvBuf) {
    int recv_length = 0;
    // Receive data from the server, recv();
    recv_length = recv(rvs_confd, recvBuf, 100, 0);
    // printf("%d\n", recv_length);
    if (recv_length <= 0)
    {
      ROS_ERROR("Recv error");
    }
    else
    {
         ROS_INFO("Recvmsg: %s\n", recvBuf);
    }
}

void FrCobotRVSCon::run() {
    //  send robot tcp to rvs
    rvsconnect();
    robot.GetActualToolFlangePose(flag, &flange);
    sprintf(rvs_send_buf, "ROBOT_TCP %f %f %f %f %f %f\n",  flange.tran.x / 1000, flange.tran.y / 1000, flange.tran.z / 1000, flange.rpy.rx / 180 * M_PI, flange.rpy.ry / 180 * M_PI, flange.rpy.rz / 180 * M_PI);
    writervs(rvs_send_buf);
    readrvs(recvTCPBuf);
    if (strcmp(recvTCPBuf, "ROBOT_TCP") == 0)
    {
        rvsdisconnect();
    } else {
        ROS_INFO("rvs_recv_buf : %s", recvTCPBuf);
    }

    //  send robot joints to rvs
    rvsconnect();
    robot.GetActualJointPosRadian(flag, &j_rad);
    sprintf(rvs_send_buf, "ROBOT_JOINTS %f %f %f %f %f %f\n", j_rad.jPos[0], j_rad.jPos[1], j_rad.jPos[2],j_rad.jPos[3], j_rad.jPos[4], j_rad.jPos[5]);
    writervs(rvs_send_buf);
    readrvs(recvJointBuf);
    if (strcmp(recvJointBuf, "ROBOT_JOINTS") == 0)
    {
        rvsdisconnect();
    } else {
        ROS_INFO("rvs_recv_buf : %s", recvJointBuf);
    }

    //  capture
    rvsconnect();
    writervs(sendCapture);
    readrvs(recvCapture);
    if (strcmp(recvCapture, "CAPTURE") == 0)
    {
        rvsdisconnect();
    } else {
        ROS_INFO("rvs_recv_buf : %s", recvCapture);
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "FrCobotRVSCon");

    FrCobotRVSCon FrCobotRVSCon;

    FrCobotRVSCon.run();

    ros::shutdown();
    return 0;
}