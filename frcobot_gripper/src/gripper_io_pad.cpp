#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QCheckBox>
#include <QDebug>

#include "frcobot_gripper/gripper_io_pad.h"
#include "frcobot_gripper/FRRobot.h"
#include "frcobot_gripper/RobotError.h"
#include "frcobot_gripper/RobotTypes.h"
#include "xmlrpc-c/base.h"
#include "xmlrpc-c/client.h"

#include <cassert>
#include <cstdlib>
#include <string>
#include <iostream>
using namespace std;

#include <XmlRpc.h>
using namespace XmlRpc;

const char * serverIP = "192.168.58.2";
const int port = 20003;  

XmlRpcClient cobotClient(serverIP, port);
XmlRpcValue noArgs, result;

namespace rviz_gripper_commander
{
    GripperPanel::GripperPanel(QWidget * parent) : rviz::Panel(parent)
    {
        QVBoxLayout *manipulate_layout = new QVBoxLayout;
        manipulate_layout->addWidget(new QLabel("XMLRPC Sever:http://192.168.58.2:20003"));

        output_gripper_open_button_ = new QPushButton("Open");
        manipulate_layout->addWidget(output_gripper_open_button_);

        output_gripper_close_button_ = new QPushButton("Close");
        manipulate_layout->addWidget(output_gripper_close_button_);

        QHBoxLayout *layout = new QHBoxLayout;
        layout->addLayout(manipulate_layout);
        setLayout(layout);

        connect(output_gripper_open_button_, SIGNAL(clicked()), this, SLOT(openGripper()));
        connect(output_gripper_close_button_, SIGNAL(clicked()), this, SLOT(closeGripper()));
    }

    void closeGripperFunc()
    {
         try {
            // Gripper closing step 1
            XmlRpcValue toolDO0;
            toolDO0[0] = 0;
            toolDO0[1] = 1;
            toolDO0[2] = 0;
            toolDO0[3] = 0;

            if (cobotClient.execute("SetToolDO", toolDO0, result))
            {
                std::cout << "Gripper closing step 1: " << int32_t(result) << "\n";
            }
            else
            {
                std::cout << "Gripper closing step 1 Failed!\n";
            }

            // Gripper closing step 2
            XmlRpcValue toolDO1;
            toolDO1[0] = 1;
            toolDO1[1] = 0;
            toolDO1[2] = 0;
            toolDO1[3] = 0;
            
            if (cobotClient.execute("SetToolDO", toolDO1, result))
            {
                std::cout << "Gripper closing step 2: " << int32_t(result) << "\n";
            }
            else
            {
                std::cout << "Gripper closing step 2 Failed!\n";
            }
        }
        catch (exception const& e)
        {
            cerr << "Client threw error: " << e.what() << endl;
        }
        catch (...)
        {
            cerr << "Client threw unexpected error." << endl;
        }
    }

    void GripperAPI::closeGripper()
    {
        closeGripperFunc();
    }

    void GripperPanel::closeGripper()
    {
        closeGripperFunc();
    }

    void openGripperFunc()
    {
        try {
            // Gripper opening step 1
            XmlRpcValue toolDO0;
            toolDO0[0] = 0;
            toolDO0[1] = 0;
            toolDO0[2] = 0;
            toolDO0[3] = 0;

            if (cobotClient.execute("SetToolDO", toolDO0, result))
            {
                std::cout << "Gripper opening step 1: " << int32_t(result) << "\n";
            }
            else
            {
                std::cout << "Gripper opening step 1 Failed!\n";
            }

            // Gripper opening step 2
            XmlRpcValue toolDO1;
            toolDO1[0] = 1;
            toolDO1[1] = 1;
            toolDO1[2] = 0;
            toolDO1[3] = 0;

            if (cobotClient.execute("SetToolDO", toolDO1, result))
            {
                std::cout << "Gripper opening step 2: " << int32_t(result) << "\n";
            }
            else
            {
                std::cout << "Gripper opening step 2 Failed!'\n";
            }
        }
        catch (exception const& e) 
        {
            cerr << "Client threw error: " << e.what() << endl;
        }
        catch (...) 
        {
            cerr << "Client threw unexpected error." << endl;
        }
    }

    void GripperPanel::openGripper()
    {
        openGripperFunc();
    }

    void GripperAPI::openGripper()
    {
        openGripperFunc();
    }

    void GripperPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    // Load all configuration data for this panel from the given Config object.
    void GripperPanel::load(const rviz::Config &config)
    {
        rviz::Panel::load(config);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_gripper_commander::GripperPanel, rviz::Panel)
