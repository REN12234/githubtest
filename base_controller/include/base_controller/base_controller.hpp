#include <ros/ros.h>
#include "base_controller/hardware_interface.hpp"
// #include "base_controller/hardware_interface_basePlusArm.hpp"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

namespace base_controller{
class base_control
{
private:
    ros::Subscriber vel_sub;
    ros::Subscriber joint_sub;
public:
    base_control();
    HardwareInterface* wheel_handler_;
    
    void init(ros::NodeHandle &nh);
     void cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg_);
    void jointvel(const std_msgs::Float64MultiArray::ConstPtr &msg);
};
};