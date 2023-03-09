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

using namespace std;

#define MAXLINE 4096
int  PORT;
string ROBOTIP;

int confd;
int send_length = 0;
int recv_length = 0;
socklen_t sendaddr_length;
char recv_buf[MAXLINE];
char send_buf[MAXLINE];
struct sockaddr_in serverSendAddr;

int intTemp = 0;
double doubleTemp = 0;
unsigned char intByte[4] = {0};
unsigned char doubleByte[8] = {0};
double tempData[6];

struct frrobot_status {
    uint8_t frame_count;
    uint8_t program_state;
    uint8_t error_code;
    uint8_t robot_mode;
    double joints_sta[6];
    double tcp_sta[6];
    double torque_sta[6];
    uint8_t tool_num;
    uint8_t cl_o_h[8];
    uint8_t cl_o_l[8];
    uint8_t tl_o_l[2];
    uint8_t robot_motion_done;
    uint8_t gripper_motion_done;
} frrobot_status;


class FrRobotStatusCtrl
{
    public:
        FrRobotStatusCtrl();
        void initTcp();
        void read();
        void update();
        void run();

    private:
        ros::NodeHandle nh_;
        ros::Publisher frrobot_status_;
};