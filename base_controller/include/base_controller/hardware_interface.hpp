#ifndef CONTROLLER_INCLUDE_HARDWARE_INTERFACE_HPP_
#define CONTROLLER_INCLUDE_HARDWARE_INTERFACE_HPP_

#include <csignal>
#include <iostream>
#include <iomanip>
#include <unistd.h>

#include "RobotMessage.h"
#include "uart.h"
#include "Loop.h"
#include <mutex>

    class HardwareInterface
    {
    private:

        RobotCmd cmd;
        RobotState  state;
        
        int _zlWheelhubMsgType;
        int _uartFrontTmotor, _uartBackTmotor, _uartFrontZLWheelhub, _uartBackZLWheelhub;
        void _initDriveTmotor();
        void _initDriveZL();
        void _eixtMotor();
        ModeState _frontModeState, _backModeState;
        ModeControl _frontModeControl, _backModeControl;
        ModeMsg _frontModeMsg, _backModeMsg;
        LoopFunc *_frontTmotorThread, *_backTmotorThread, *_frontZLWheelhubThread, *_backZLWheelhubThread;
        std::mutex _sendrecv;

        

    public:
        HardwareInterface();      
        ~HardwareInterface();        

        bool sendRecv(RobotCmd *cmd, RobotState *state);
        
        void getLeftPosition();
        void getRightPosition();
        void getLeftVelocity();
        void getRightVelocity();
        void respectiveCommand(float front_left_vel,float front_right_vel, float back_left_vel,float back_right_vel,
                                                                float front_left_ang,float front_right_ang, float back_left_ang,float back_right_ang);
        // void respectiveAngle(float front_left_vel,float front_right_vel, float back_left_vel,float back_right_vel);

        // void turnningControl(float angular);
    };
#endif // CONTROLLER_INCLUDE_HARDWARE_INTERFACE_HPP_

