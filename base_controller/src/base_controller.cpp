#include "base_controller/base_controller.hpp"

namespace base_controller{
base_control::base_control()
{}
void base_control::init(ros::NodeHandle &nh)
{
    wheel_handler_ = new HardwareInterface();
    joint_sub = nh.subscribe<std_msgs::Float64MultiArray>("/swerve_joints_control", 1, &base_control::jointvel, this);
    vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1,&base_control::cmdVelCB,this);
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
        double value7 = input_values[6];
        double value8 = input_values[7];
        wheel_handler_->respectiveCommand(value1,value2,value3,value4,value5,value6,value7,value8);
     
}

void base_control::cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg_)
{
    wheel_handler_->respectiveCommand(msg_->linear.x,msg_->linear.x,msg_->linear.x,msg_->linear.x,
                                                                                    msg_->angular.z,msg_->angular.z,msg_->angular.z,msg_->angular.z);
    // std::cout<<"bug"<<msg_->linear.x<<std::endl;
    // std::cout<<"test"<<msg_->angular.z<<std::endl;
    wheel_handler_->getRightVelocity();
    wheel_handler_->getLeftVelocity();
    wheel_handler_->getLeftPosition();
    wheel_handler_->getRightPosition();
}
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "base_control_node");
    ros::NodeHandle nh;
    ros::Time begin=ros::Time::now();   
    
    base_controller::base_control bc;
    bc.init(nh);
  
    ros::Rate looprate(10);
   
    
    ros::spin();
    ros::shutdown();
    return 0;
}
