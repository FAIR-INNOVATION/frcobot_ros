
#include "frcobot_camera/frcobot_rvs.h"

FrCobotRVSCon::FrCobotRVSCon() {
    if (nh.hasParam("rvs_ip")) {
        nh.getParam("rvs_ip", RVS_IP);
        nh.getParam("rvs_port", RVS_PORT);
        rvsIP = (char *)RVS_IP.c_str();
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

    memset(&epos, 0, sizeof(ExaxisPos));
    memset(&offset_pose, 0, sizeof(DescPose));
    memset(&p1Pose, 0, sizeof(DescPose));
    memset(&p2Pose, 0, sizeof(DescPose));
    memset(&p1ActPose, 0, sizeof(DescPose));
    memset(&p2ActPose, 0, sizeof(DescPose));
    memset(&p1PrePose, 0, sizeof(DescPose));
    memset(&p2PrePose, 0, sizeof(DescPose));
    memset(&p1PostPose, 0, sizeof(DescPose));
    memset(&p2PostPose, 0, sizeof(DescPose));
    memset(&p1Joint, 0, sizeof(JointPos));
    memset(&p2Joint, 0, sizeof(JointPos));
    memset(&p1ActJoint, 0, sizeof(JointPos));
    memset(&p2ActJoint, 0, sizeof(JointPos));
    memset(&p1PreJoint, 0, sizeof(JointPos));
    memset(&p2PreJoint, 0, sizeof(JointPos));
    memset(&p1PostJoint, 0, sizeof(JointPos));
    memset(&p2PostJoint, 0, sizeof(JointPos));

    // Initialize the pHome point (photograph point)
    memset(&pHomePose , 0, sizeof(DescPose));
    memset(&pHomeJoint, 0, sizeof(JointPos));

    nh.getParam("pHome/x", pHomePose.tran.x);
    nh.getParam("pHome/y", pHomePose.tran.y);
    nh.getParam("pHome/z", pHomePose.tran.z);
    nh.getParam("pHome/rx", pHomePose.rpy.rx);
    nh.getParam("pHome/ry", pHomePose.rpy.ry);
    nh.getParam("pHome/rz", pHomePose.rpy.rz);
    nh.getParam("pHome/j1", pHomeJoint.jPos[0]);
    nh.getParam("pHome/j2", pHomeJoint.jPos[1]);
    nh.getParam("pHome/j3", pHomeJoint.jPos[2]);
    nh.getParam("pHome/j4", pHomeJoint.jPos[3]);
    nh.getParam("pHome/j5", pHomeJoint.jPos[4]);
    nh.getParam("pHome/j6", pHomeJoint.jPos[5]);
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

    robot.GetActualTCPNum(flag, &toolCoordNum);
    printf("Tool coord num:%d\n", toolCoordNum);

    int ret1 = robot.MoveJ(&pHomeJoint, &pHomePose, toolCoordNum, user, vel, acc, ovl, &epos, blendT, flag, &offset_pose);
    printf("MoveJ to pHome:%d\n", ret1);

    //  send robot tcp to rvs
    rvsconnect();
    robot.GetActualToolFlangePose(flag, &flange);
    sprintf(rvs_send_buf, "ROBOT_TCP %f %f %f %f %f %f\n",  flange.tran.x / 1000, 
                                                            flange.tran.y / 1000, 
                                                            flange.tran.z / 1000, 
                                                            flange.rpy.rx / 180 * M_PI, 
                                                            flange.rpy.ry / 180 * M_PI, 
                                                            flange.rpy.rz / 180 * M_PI);
    writervs(rvs_send_buf);
    readrvs(recvTCPBuf);
    if (strcmp(recvTCPBuf, "ROBOT_TCP") == 0)
    {
        rvsdisconnect();
    } else {
        ROS_INFO("rvs_recv_buf : %s", recvTCPBuf);
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

    //  get poses from rvs
    rvsconnect();
    writervs(sendGetPosesCmd);
    readrvs(recvPosesBuf);
    char *result = strtok(recvPosesBuf, delim);
    vector<double> res_split;
    while(result != NULL){
        res_split.push_back(strtod(result, NULL));
        // printf( "%s\n", result );
        result = strtok( NULL, delim );
    }
    //  grab point
    p1Pose.tran.x = res_split[2] * 1000;
    p1Pose.tran.y = res_split[3] * 1000;
    p1Pose.tran.z = res_split[4] * 1000;
    p1Pose.rpy.rx = res_split[5] / M_PI * 180;
    p1Pose.rpy.ry = res_split[6] / M_PI * 180;
    p1Pose.rpy.rz = res_split[7] / M_PI * 180;
    printf("p1Pose:%f,%f,%f,%f,%f,%f\n",p1Pose.tran.x,p1Pose.tran.y,p1Pose.tran.z,p1Pose.rpy.rx,p1Pose.rpy.ry,p1Pose.rpy.rz);

    //  placement point
    p2Pose.tran.x = res_split[8] * 1000;
    p2Pose.tran.y = res_split[9] * 1000;
    p2Pose.tran.z = res_split[10] * 1000;
    p2Pose.rpy.rx = res_split[11] / M_PI * 180;
    p2Pose.rpy.ry = res_split[12] / M_PI * 180;
    p2Pose.rpy.rz = res_split[13] / M_PI * 180;
    printf("p2Pose:%f,%f,%f,%f,%f,%f\n",p2Pose.tran.x,p2Pose.tran.y,p2Pose.tran.z,p2Pose.rpy.rx,p2Pose.rpy.ry,p2Pose.rpy.rz);

    robot.ComputePrePick(&p1Pose, prepickzlength, prepickzangle, &p1PrePose);
    robot.GetInverseKin(type, &p1PrePose, -1, &p1PreJoint);

    robot.ComputePrePick(&p2Pose, prepickzlength, prepickzangle, &p2PrePose);
    robot.GetInverseKin(type, &p2PrePose, -1, &p2PreJoint);

    int ret2 = robot.MoveJ(&p1PreJoint, &p1PrePose, toolCoordNum, user, vel, acc, ovl, &epos, blendT, flag, &offset_pose);
    printf("MoveJ to p1Pre:%d\n", ret2);

    robot.GetInverseKin(type, &p1Pose, -1, &p1Joint);
    int ret3 = robot.MoveL(&p1Joint, &p1Pose, toolCoordNum, user, vel, acc, ovl, blendR, &epos, search_flag, flag, &offset_pose);
    printf("MoveL to p1:%d\n", ret3);

    robot.ComputePostPick(&p1Pose, retreatzlength, retreatzangle, &p1PostPose);
    robot.GetInverseKin(type, &p1PostPose, -1, &p1PostJoint);
    robot.SetDO(0, status_0, smooth, block);
    sleep(1);
    int ret4 = robot.MoveL(&p1PostJoint, &p1PostPose, toolCoordNum, user, vel, acc, ovl, blendR, &epos, search_flag, flag, &offset_pose);
    printf("MoveL to p1Post:%d\n", ret4);

    printf("cube_count: %d\n", cube_count);
    printf("layer: %d\n", layer);
    if (cube_count == 0)
    {
        offset_flag = 1;
        offset_pose.tran.x = 0;
        offset_pose.tran.y = 0;
        offset_pose.tran.z = 50 * layer;
    } 
    else if (cube_count == 1)
    {
        offset_flag = 1;
        offset_pose.tran.x = 40;
        offset_pose.tran.y = -35;
        offset_pose.tran.z = 50 * layer;
    }

    int ret5 = robot.MoveJ(&p2PreJoint, &p2PrePose, toolCoordNum, user, vel, acc, ovl, &epos, blendT, offset_flag, &offset_pose);
    printf("MoveJ to p2Pre:%d\n", ret5);

    robot.GetInverseKin(type, &p2Pose, -1, &p2Joint);
    int ret6 = robot.MoveL(&p2Joint, &p2Pose, toolCoordNum, user, vel, acc, ovl, blendR, &epos, search_flag, offset_flag, &offset_pose);
    printf("MoveL to p2:%d\n", ret6);

    robot.ComputePostPick(&p2Pose, retreatzlength, retreatzangle, &p2PostPose);
    robot.GetInverseKin(type, &p2PostPose, -1, &p2PostJoint);
    robot.SetDO(0, status_1, smooth, block);
    sleep(1);
    int ret7 = robot.MoveL(&p2PostJoint, &p2PostPose, toolCoordNum, user, vel, acc, ovl, blendR, &epos, search_flag, offset_flag, &offset_pose);
    printf("MoveL to p2Post:%d\n", ret7);

    if (cube_count == 0)
    {
        cube_count = 1;
    }
    else
    {
        cube_count = 0;
        layer++;
    }

    rvsdisconnect();

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "FrCobotRVSCon");

    FrCobotRVSCon FrCobotRVSCon;

    while(ros::ok()) {
        FrCobotRVSCon.run();
    }

    return 0;
}