
// #include <unistd.h>
// #include <cstdlib>
// #include <iostream>
// #include <stdio.h>
// #include <cstring>
// #include <unistd.h>
#include "frcobot_hw/frcobot_hw.h"
#include "frcobot_hw/FRRobot.h"
#include "frcobot_hw/RobotError.h"
#include "frcobot_hw/RobotTypes.h"

#include "xmlrpc-c/base.h"
#include "xmlrpc-c/client.h"

using namespace std;

int main(int argc, char** argv)
{
    FRRobot robot;                 //Instantiate the robot object
    robot.RPC("192.168.58.2");     //Establish a communication connection with the robot controller

    char ip[64]="";
    char version[64] = "";
    uint8_t state;
    int id;
    int  type = 0;
    DescPose flange, desc_pose, pre_pose, post_pose, offset_pose;
    JointPos j_rad, j_pose_inver;
    ExaxisPos epos;

    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 100.0;
    float blendT = 0.0;
    float blendR = 0.0;
    uint8_t flag = 0;
    uint8_t search = 0;

    uint8_t status_0 = 0; 
    uint8_t status_1 = 1; 
    uint8_t smooth = 0;
    uint8_t block = 0;

    memset(&epos, 0, sizeof(ExaxisPos));
    memset(&offset_pose, 0, sizeof(DescPose));

    // robot.GetSDKVersion(version);
    // printf("SDK version:%s\n", version);
    // robot.GetControllerIP(ip);
    // printf("controller ip:%s\n", ip);

    // auto mode
    robot.Mode(0);
    sleep(1);

    robot.GetActualToolFlangePose(flag, &flange);
    printf("tcp pose : %f, %f, %f, %f, %f, %f\n", flange.tran.x, flange.tran.y,  flange.tran.z, flange.rpy.rx, flange.rpy.ry, flange.rpy.rz);

    robot.GetActualJointPosRadian(flag, &j_rad);
    printf("joint pos rad:%f,%f,%f,%f,%f,%f\n", j_rad.jPos[0],j_rad.jPos[1],j_rad.jPos[2],j_rad.jPos[3],j_rad.jPos[4],j_rad.jPos[5]);

    robot.GetActualTCPNum(flag, &id);
    printf("tcp num:%d\n", id);

    int  ret = robot.GetInverseKin(type, &flange, -1, &j_pose_inver);
    printf("joint pos InverseKin:%d,%f,%f,%f,%f,%f,%f\n", ret, j_pose_inver.jPos[0],j_pose_inver.jPos[1],j_pose_inver.jPos[2],j_pose_inver.jPos[3],j_pose_inver.jPos[4],j_pose_inver.jPos[5]);

    robot.GetForwardKin(&j_pose_inver, &desc_pose);
    printf("desc_pose tcp pose : %f, %f, %f, %f, %f, %f\n", desc_pose.tran.x, desc_pose.tran.y,  desc_pose.tran.z, desc_pose.rpy.rx, desc_pose.rpy.ry, desc_pose.rpy.rz);

    robot.ComputePrePick(&desc_pose, -100, 0, &pre_pose);
    printf("pre_pose tcp pose : %f, %f, %f, %f, %f, %f\n", pre_pose.tran.x, pre_pose.tran.y,  pre_pose.tran.z, pre_pose.rpy.rx, pre_pose.rpy.ry, pre_pose.rpy.rz);

    robot.ComputePostPick(&desc_pose, -100, 0, &post_pose);
    printf("post_pose tcp pose : %f, %f, %f, %f, %f, %f\n", post_pose.tran.x, post_pose.tran.y,  post_pose.tran.z, post_pose.rpy.rx, post_pose.rpy.ry, post_pose.rpy.rz);

    int err1 = robot.MoveJ(&j_pose_inver, &flange, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pose);
    printf("movej code:%d\n", err1);

    int err2 = robot.MoveL(&j_pose_inver, &flange, tool, user, vel, acc, ovl, blendR, &epos,search,flag, &offset_pose);
    printf("movel code:%d\n", err2);

    robot.SetToolDO(0, status_1, smooth, block);
    robot.SetToolDO(1, status_0, smooth, block);
    sleep(3);
    robot.SetToolDO(0, status_0, smooth, block);
    robot.SetToolDO(1, status_1, smooth, block);
    // manual mode
    robot.Mode(1);

    return 0;
}