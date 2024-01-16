#ifndef CONTROLLER_INCLUDE_HARDWARE_INTERFACE_ARM_HPP_
#define CONTROLLER_INCLUDE_HARDWARE_INTERFACE_ARM_HPP_

#include <csignal>
#include <iostream>
#include <iomanip>
#include <unistd.h>

#include "RobotMessage.h"
#include "uart.h"
#include "Loop.h"
#include <mutex>
#include "base_controller_arm/KortexRobot.h"


#define UART_PATH_FRONT_TMOTOR (char *)"/dev/ttyACM0"
#define UART_PATH_BACK_TMOTOR (char *)"/dev/ttyACM2"
#define UART_PATH_FRONT_ZL (char *)"/dev/ttyACM1"
#define UART_PATH_BACK_ZL (char *)"/dev/ttyACM3"
#define BAUD 115200

class HardwareInterfaceArm
{
private:
    int _zlWheelhubMsgType;

    RobotCmd cmd;
    RobotState state;
    int _uartFrontTmotor, _uartBackTmotor, _uartFrontZLWheelhub, _uartBackZLWheelhub;
    KortexRobot *_robot;
    ModeState _frontModeState, _backModeState;
    ModeControl _frontModeControl, _backModeControl;
    ModeMsg _frontModeMsg, _backModeMsg;
    float _cmd_x, _cmd_y, _cmd_z, _cmd_theta_x, _cmd_theta_y, _cmd_theta_z;
    float _state_x, _state_y, _state_z, _state_theta_x, _state_theta_y, _state_theta_z;
    LoopFunc *_frontTmotorThread, *_backTmotorThread, *_frontZLWheelhubThread, *_backZLWheelhubThread, *_kinovaArmThread;
    std::mutex _sendrecv;
    
    //机械臂
    std::vector<float> speeds;
    std::vector<float> jointVelocities;
    std::vector<float> jointPositions;

    void _initDriveZL();
    void _initDriveTmotor();
    void _eixtMotor();

public:
    HardwareInterfaceArm();
    ~HardwareInterfaceArm();

    bool sendRecv(RobotCmd *cmd, RobotState *state);
    void etherCatSendRecvKinova();
    
    void getLeftPosition();
    void getRightPosition();
    void getLeftVelocity();
    void getRightVelocity();
    void respectiveCommand(float front_left_vel,float front_right_vel, float back_left_vel,float back_right_vel,
                            float front_left_ang,float front_right_ang, float back_left_ang,float back_right_ang);
    void respectivearmCommand(float arm1, float arm2, double arm3, double arm4, double arm5, double arm6);

    // void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    // void read(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    
    // IORealRobot node_;

};
#endif // CONTROLLER_INCLUDE_HARDWARE_INTERFACE_HPP_

