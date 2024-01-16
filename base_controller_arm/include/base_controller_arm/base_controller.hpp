#include <ros/ros.h>
#include "base_controller_arm/hardware_interface_arm.hpp"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h> // 包含所需的头文件
#include "RobotMessage.h"

namespace base_controller_arm{
class base_control
{
private:
    ros::Subscriber vel_sub;
    ros::Subscriber joint_sub;
public:
    base_control();
    HardwareInterfaceArm* wheel_handler_;
    
    void init(ros::NodeHandle &nh);
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg_);
    //  void jointvel(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void jointvel(const std_msgs::Float64MultiArray::ConstPtr &msg);
 
};

}