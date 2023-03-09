#ifndef GRIPPER_IO_PAD_H
#define GRIPPER_IO_PAD_H

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>

class QLineEdit;            //inputline edit
class QPushButton;    // qt button

namespace rviz_gripper_commander
{
    class GripperPanel : public rviz::Panel
    {
        Q_OBJECT
        public:
            GripperPanel(QWidget* parent  = 0);

            virtual void load (const rviz::Config& config);
            virtual void save (rviz::Config config) const;

        public Q_SLOTS:
            void openGripper();
            void closeGripper();

        protected:
            QString output_topic_;
            QPushButton* output_gripper_open_button_;
            QPushButton* output_gripper_close_button_;
    };

    class GripperAPI
    {
        public:
            void openGripper();
            void closeGripper();
    };
}

#endif
