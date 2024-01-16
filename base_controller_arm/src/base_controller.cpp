#include "base_controller_arm/base_controller.hpp"

namespace base_controller_arm{
base_control::base_control()
{}
void base_control::init(ros::NodeHandle &nh)
{
    wheel_handler_ = new HardwareInterfaceArm();
    vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10,&base_control::cmdVelCB,this);
    
    joint_sub = nh.subscribe<std_msgs::Float64MultiArray>("/swerve_joints_control", 10, &base_control::jointvel, this);
}

void base_control:: cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg_)
{
    wheel_handler_->respectiveCommand(0,0,0,0,0,0,0,0);
    // std::cout<< "bug"<<linear.x<<std::endl;
    // std::cout<<" bug2"<<angular.z<<std::endl;
    wheel_handler_->getRightVelocity();
    wheel_handler_->getLeftVelocity();
    wheel_handler_->getLeftPosition();
    wheel_handler_->getRightPosition();

}


void base_control::jointvel(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    std::vector<double> input_values = msg->data;

        double value1 = input_values[0];
        double value2 = input_values[1];
        double value3 = input_values[2];
        double value4 = input_values[3];
        double value5 = input_values[4];
        double value6 = input_values[5];
        wheel_handler_->respectivearmCommand(value1,value2,value3,value4,value5,value6);
        // wheel_handler_->respectivearmCommand(0,0,0,0,0,-5);
}
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "base_control_node");
    ros::NodeHandle nh;
    base_controller_arm::base_control bc;
    bc.init(nh);
    ros::Rate looprate(10);
    // bc.jointvel();
    ros::spin();
    ros::shutdown();
    return 0;
}
