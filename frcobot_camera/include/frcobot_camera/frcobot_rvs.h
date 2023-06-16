#include <ros/ros.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <math.h>
#include <string>
#include <iostream>
#include <vector>

#include "frcobot_camera/FRRobot.h"
#include "frcobot_camera/RobotError.h"
#include "frcobot_camera/RobotTypes.h"

#include "xmlrpc-c/base.h"
#include "xmlrpc-c/client.h"

using namespace std;

#define _USE_MATH_DEFINES
#define MAXLINE 4096

int RVS_PORT;
string RVS_IP;
const char *rvsIP;

int rvs_confd;
socklen_t rvs_sendaddr_length;
char rvs_recv_buf[MAXLINE] = "";
char rvs_send_buf[MAXLINE] = "";
struct sockaddr_in rvsserverSendAddr;

FRRobot robot;                 //Instantiate the robot object
string ROBOTIP;
const char *robotIP;

int toolCoordNum;
int cube_count = 0;
int layer = 0;
uint8_t user = 0;
uint8_t type = 0;
float vel = 50.0;
float acc = 50.0;
float ovl = 50.0;
float blendT = 0.0;
float blendR = 0.0;
uint8_t flag = 0;
uint8_t offset_flag = 0;
uint8_t search_flag = 0;

uint8_t status_0 = 0; 
uint8_t status_1 = 1; 
uint8_t smooth = 0;
uint8_t block = 0;

int prepickzlength = -200;           // The z-axis offset of the grab point, based on the tool coordinate system
int prepickzangle = 0;               // The z-axis offset of the grab point, based on the tool coordinate system
int retreatzlength = -200;           // The z-axis offset of the placement point, based on the tool coordinate system
int retreatzangle = 0;               // The z-axis offset of the placement point, based on the tool coordinate system

DescPose flange, pHomePose, offset_pose, p1Pose, p2Pose, p1ActPose, p2ActPose, p1PrePose, p2PrePose, p1PostPose, p2PostPose; 
JointPos j_rad, pHomeJoint, p1Joint, p2Joint, p1ActJoint, p2ActJoint, p1PreJoint, p1PostJoint, p2PreJoint, p2PostJoint;
ExaxisPos epos;

char recvTCPBuf[MAXLINE] = "";
char sendCapture[MAXLINE] = "CAPTURE \n";
char recvCapture[MAXLINE] = "";
char sendGetPosesCmd[MAXLINE] = "GET_POSES \n";
char recvPosesBuf[MAXLINE ]= "";
char recvJointBuf[MAXLINE] = "";
const char delim[] = " ";

class FrCobotRVSCon
{
    public:
        FrCobotRVSCon();
        void rvsconnect();
        void rvsdisconnect();
        void writervs(const char* sendBuf);
        void readrvs(char * recvBuf);
        void run();

    private:
        ros::NodeHandle nh;
};