#include "base_controller_arm/hardware_interface_arm.hpp"

    HardwareInterfaceArm::HardwareInterfaceArm()
    {
    _zlWheelhubMsgType = 0;

    _uartFrontTmotor = uart_init(BAUD, UART_PATH_FRONT_TMOTOR);
    _uartBackTmotor = uart_init(BAUD, UART_PATH_BACK_TMOTOR);
    _uartFrontZLWheelhub = uart_init(BAUD, UART_PATH_FRONT_ZL);
    _uartBackZLWheelhub = uart_init(BAUD, UART_PATH_BACK_ZL);

    initMsg(&_frontModeMsg);
    initMsg(&_backModeMsg);

    _initDriveTmotor();
    _initDriveZL();

    _robot = new KortexRobot("192.168.1.10");
    if (_robot->IsConnected())
    {
        std::cout << "Connection successful" << std::endl;
        _robot->SubscribeToNotifications();
        _cmd_x = 0;
        _cmd_y = 0;
        _cmd_z = 1.29;
        _cmd_theta_x = 0;
        _cmd_theta_y = 0;
        _cmd_theta_z = 0;
    }
    else
    {
        std::cout << "Connection faild" << std::endl;
    }

    _frontTmotorThread = new LoopFunc("front Tmotor sendrecv thread", 0.002, boost::bind(&uartSendRecvTmotor, _uartFrontTmotor, &_frontModeMsg, &_frontModeState));
    _backTmotorThread = new LoopFunc("back Tmotor sendrecv thread", 0.002, boost::bind(&uartSendRecvTmotor, _uartBackTmotor, &_backModeMsg, &_backModeState));
    _frontZLWheelhubThread = new LoopFunc("front zl wheelhub sendrecv thread", 0.002, boost::bind(&uartSendRecvZLWheelhub, _uartFrontZLWheelhub, &_frontModeMsg, &_frontModeState));
    _backZLWheelhubThread = new LoopFunc("back zl wheelhub sendrecv thread", 0.002, boost::bind(&uartSendRecvZLWheelhub, _uartBackZLWheelhub, &_backModeMsg, &_backModeState));
    _kinovaArmThread = new LoopFunc("kinova arm sendrecv thread", 0.05, boost::bind(&HardwareInterfaceArm::etherCatSendRecvKinova, this));
     
    _frontTmotorThread->start();
    _backTmotorThread->start();
    _frontZLWheelhubThread->start();
    _backZLWheelhubThread->start();
    _kinovaArmThread->start();


}

HardwareInterfaceArm::~HardwareInterfaceArm(){
    uart_close(_uartFrontTmotor);
    uart_close(_uartBackTmotor);
    uart_close(_uartFrontZLWheelhub);
    uart_close(_uartBackZLWheelhub);
    
    _robot->UnsubscribeToNotifications();

    delete _frontTmotorThread;
    delete _backTmotorThread;
    delete _frontZLWheelhubThread;
    delete _backZLWheelhubThread;
    delete _robot;
    std::cout << "uart stopped! " << std::endl;
}

void HardwareInterfaceArm::_eixtMotor(){
    exitMotorModeCmdTmotor(&_frontModeMsg.TmotorLeftMsg);
    exitMotorModeCmdTmotor(&_frontModeMsg.TmotorRightMsg);

    exitMotorModeCmdTmotor(&_backModeMsg.TmotorLeftMsg);
    exitMotorModeCmdTmotor(&_backModeMsg.TmotorRightMsg);

    exitMotorModeCmdZLWheelhub(&_frontModeMsg.ZLWheelhubMsg);
    exitMotorModeCmdZLWheelhub(&_backModeMsg.ZLWheelhubMsg);

    uartSendTmotor(_uartFrontTmotor, &_frontModeMsg);
    uartSendTmotor(_uartBackTmotor, &_backModeMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);
}

void HardwareInterfaceArm::_initDriveTmotor(){
    enterMotorModeCmdTmotor(&_frontModeMsg.TmotorLeftMsg);
    enterMotorModeCmdTmotor(&_frontModeMsg.TmotorRightMsg);
    uartSendTmotor(_uartFrontTmotor, &_frontModeMsg);

    enterMotorModeCmdTmotor(&_backModeMsg.TmotorLeftMsg);
    enterMotorModeCmdTmotor(&_backModeMsg.TmotorRightMsg);
    uartSendTmotor(_uartBackTmotor, &_backModeMsg);
}

void HardwareInterfaceArm::_initDriveZL(){
    prepareWheelHubDrive1(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive1(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive2(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive2(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive3(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive3(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive4(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive4(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setSynchriniusControl4Wheelhub(&_frontModeMsg.ZLWheelhubMsg);
    setSynchriniusControl4Wheelhub(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setVelocityMode4Wheelhub(&_frontModeMsg.ZLWheelhubMsg);
    setVelocityMode4Wheelhub(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setLeftWheelhubAccTime(&_frontModeMsg.ZLWheelhubMsg);
    setLeftWheelhubAccTime(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setRightWheelhubAccTime(&_frontModeMsg.ZLWheelhubMsg);
    setRightWheelhubAccTime(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setLeftWheelhubDecTime(&_frontModeMsg.ZLWheelhubMsg);
    setLeftWheelhubDecTime(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setRightWheelhubDecTime(&_frontModeMsg.ZLWheelhubMsg);
    setRightWheelhubDecTime(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive2(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive2(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive3(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive3(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive4(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive4(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);
}

bool HardwareInterfaceArm::sendRecv(RobotCmd *cmd, RobotState *state)
{
    // 0 & 6 front Tmotor; 1 & 7 front ZLWheelhub
    // 2 & 4 back Tmotor; 3 & 5 back ZLWheelhub

    _frontModeControl.TmotorLeftCtrl.kp = cmd->kp[0];
    _frontModeControl.TmotorLeftCtrl.p_des = cmd->q[0];
    _frontModeControl.TmotorLeftCtrl.kd = cmd->kd[0];
    _frontModeControl.TmotorLeftCtrl.v_des = cmd->qd[0];
    _frontModeControl.TmotorLeftCtrl.t_ff = cmd->tau[0];

    _frontModeControl.TmotorRightCtrl.kp = cmd->kp[6];
    _frontModeControl.TmotorRightCtrl.p_des = cmd->q[6];
    _frontModeControl.TmotorRightCtrl.kd = cmd->kd[6];
    _frontModeControl.TmotorRightCtrl.v_des = cmd->qd[6];
    _frontModeControl.TmotorRightCtrl.t_ff = cmd->tau[6];

    _frontModeControl.ZLWheelhubLeftCtrl.v_des = cmd->qd[1];
    _frontModeControl.ZLWheelhubRightCtrl.v_des = cmd->qd[7];

    _backModeControl.TmotorLeftCtrl.kp = cmd->kp[2];
    _backModeControl.TmotorLeftCtrl.p_des = cmd->q[2];
    _backModeControl.TmotorLeftCtrl.kd = cmd->kd[2];
    _backModeControl.TmotorLeftCtrl.v_des = cmd->qd[2];
    _backModeControl.TmotorLeftCtrl.t_ff = cmd->tau[2];

    _backModeControl.TmotorRightCtrl.kp = cmd->kp[4];
    _backModeControl.TmotorRightCtrl.p_des = cmd->q[4];
    _backModeControl.TmotorRightCtrl.kd = cmd->kd[4];
    _backModeControl.TmotorRightCtrl.v_des = cmd->qd[4];
    _backModeControl.TmotorRightCtrl.t_ff = cmd->tau[4];

    _backModeControl.ZLWheelhubLeftCtrl.v_des = cmd->qd[3];
    _backModeControl.ZLWheelhubRightCtrl.v_des = cmd->qd[5] ;

    if (_zlWheelhubMsgType < 4)
    {
        ++_zlWheelhubMsgType;
    }
    else if (_zlWheelhubMsgType >= 4)
    {
        _zlWheelhubMsgType = 0;
    }

    //  _sendrecv.lock();
    packAllCmdTmotor(&_frontModeMsg, _frontModeControl);
    packAllCmdTmotor(&_backModeMsg, _backModeControl);
    if (_zlWheelhubMsgType == 0)
    {
        packAllCmdZLWheelhub(&_frontModeMsg, _frontModeControl);
        packAllCmdZLWheelhub(&_backModeMsg, _backModeControl);
    }
    else if (_zlWheelhubMsgType == 1)
    {
        readWheelhubVelocity(&_frontModeMsg.ZLWheelhubMsg);
        readWheelhubVelocity(&_backModeMsg.ZLWheelhubMsg);
    }
    else if (_zlWheelhubMsgType == 2)
    {
        readLeftWheelhubPosition(&_frontModeMsg.ZLWheelhubMsg);
        readLeftWheelhubPosition(&_backModeMsg.ZLWheelhubMsg);
    }
    else if (_zlWheelhubMsgType == 3)
    {
        readRightWheelhubPosition(&_frontModeMsg.ZLWheelhubMsg);
        readRightWheelhubPosition(&_backModeMsg.ZLWheelhubMsg);
    }
    _sendrecv.unlock();

    state->q[0] = _frontModeState.TmotorLeftState.p;
    state->qd[0] = _frontModeState.TmotorLeftState.v;
    state->tau[0] = _frontModeState.TmotorLeftState.tau;

    state->q[1] = _frontModeState.ZLWheelhubLeftState.p;
    state->qd[1] = _frontModeState.ZLWheelhubLeftState.v;

    state->q[2] = _backModeState.TmotorLeftState.p;
    state->qd[2] = _backModeState.TmotorLeftState.v;
    state->tau[2] = _backModeState.TmotorLeftState.tau;

    state->q[3] = _backModeState.ZLWheelhubLeftState.p;
    state->qd[3] = _backModeState.ZLWheelhubLeftState.v;

    state->q[4] = _backModeState.TmotorRightState.p;
    state->qd[4] = _backModeState.TmotorRightState.v;
    state->tau[4] = _backModeState.TmotorRightState.tau;

    state->q[5] = _backModeState.ZLWheelhubRightState.p;
    state->qd[5] = _backModeState.ZLWheelhubRightState.v;

    state->q[6] = _frontModeState.TmotorRightState.p;
    state->qd[6] = _frontModeState.TmotorRightState.v;
    state->tau[6] = _frontModeState.TmotorRightState.tau;

    state->q[7] = _frontModeState.ZLWheelhubRightState.p;
    state->qd[7] = _frontModeState.ZLWheelhubRightState.v;
    return true;
}

void HardwareInterfaceArm::getLeftPosition()
{

    float front_left_pos_ = state.q[1];
    float back_left_pos_ = state.q[3];
    std::cout << "front_left_pos_: " << front_left_pos_ << std::endl;
    std::cout << "back_left_pos_: " << back_left_pos_ << std::endl;
}

void HardwareInterfaceArm::getRightPosition()
{

    float front_right_pos_ = state.q[7];
    float back_right_pos_ = state.q[5];
    std::cout << "front_right_pos_: " << front_right_pos_ << std::endl;
    std::cout << "back_right_pos_: " << back_right_pos_ << std::endl;
    std::cout << "             " << std::endl;
}

void HardwareInterfaceArm::getLeftVelocity()
{

    float front_left_vel_ = state.qd[1] * M_PI * 0.14 /60;
    float back_left_vel_ = state.qd[3] * M_PI * 0.14 /60;
    std::cout << "front_left_vel_: " << front_left_vel_ << std::endl;
    std::cout << "back_left_vel_: " << back_left_vel_ << std::endl;
}

void HardwareInterfaceArm::getRightVelocity()
{
    float front_right_vel_ = state.qd[7] * M_PI * 0.14 /60;
    float back_right_vel_ = state.qd[5] * M_PI * 0.14 /60;
    std::cout << "front_right_vel_: " << front_right_vel_ << std::endl;
    std::cout << "back_right_vel_: " << back_right_vel_ << std::endl;
}
void HardwareInterfaceArm::respectiveCommand(float front_left_vel, float front_right_vel, float back_left_vel, float back_right_vel,
                                          float front_left_ang, float front_right_ang, float back_left_ang, float back_right_ang)
{
    double _maxTurningSpeed = 1;
    cmd.qd[1] = front_left_vel   * 60 / 3.14 /0.14;
    cmd.qd[7] = front_right_vel  * 60 / 3.14 /0.14;
    cmd.qd[3] = -back_left_vel * 60 / 3.14 /0.14;
    cmd.qd[5] = back_right_vel  * 60 / 3.14 /0.14;

    //位置控制模式
    // double kp[8] = {20, 0, 20, 0, 20, 0, 20, 0};
    // double kd[8] = {.1, 5, .1, 5, .1, 5, .1, 5};
    // for (int i = 0; i < 8; i++)
    // {
    //     cmd.kp(i) = kp[i];
    //     cmd.kd(i) = kd[i];
    // }
    // cmd.q[0] = front_left_ang;
    // cmd.q[6] = front_right_ang;
    // cmd.q[2] = back_left_ang;
    // cmd.q[4] = back_right_ang;

    //速度控制模式
    // 转向电机的位置比例、微分系数
    double kp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double kd[8] = {5, 5, 5, 5, 5, 5, 5, 5};
    //  double kd[8] = {.1, 5, .1, 5, .1, 5, .1, 5};
    for (int i = 0; i < 8; i++)
    {
        cmd.kp(i) = kp[i];
        cmd.kd(i) = kd[i];
    }
    double _direction = 0;
    double _maxDirectionChangeRate = 0.03;
    double _maxYawAngle = 0.785;


    cmd.qd[0] = front_left_ang;
    cmd.qd[6] = front_right_ang;
    cmd.qd[2] = back_left_ang;
    cmd.qd[4] = back_right_ang;

    // sendRecv(&cmd, &state);
}

void HardwareInterfaceArm::etherCatSendRecvKinova()
{
    if(_robot->IsConnected())
    {   
        Kinova::Api::Base::JointSpeeds joint_speeds;
        Kinova::Api::Base::CartesianTrajectoryConstraint constraint = Kinova::Api::Base::CartesianTrajectoryConstraint();
        constraint.mutable_speed()->set_translation(0.25f);
        constraint.mutable_speed()->set_orientation(30.0f);

        for (size_t i = 0 ; i < speeds.size(); ++i)
        {
            auto joint_speed = joint_speeds.add_joint_speeds();
            joint_speed->set_joint_identifier(i);
            joint_speed->set_value(speeds.at(i));
            joint_speed->set_duration(1);
        }
        // std::cout<<"bug"<<std::endl;
        bool d = _robot->SetJointSpeeds(speeds);
        std::cout<<"bug"<<d<<std::endl;
        _robot->RefreshFeedback();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::vector<float> jointTorques = _robot->GetJointTorques();
        if(!_robot->SetCustomData(jointTorques))
        {
            std::cout << "Custom data has an invalid size" << std::endl; 
        }
        jointVelocities = _robot->GetJointVelocities();
        jointPositions = _robot->GetJointPositions();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
void HardwareInterfaceArm::respectivearmCommand(float arm1,float arm2,double arm3,double arm4,double arm5,double arm6) 
{   
    speeds = {arm1,arm2,arm3,arm4,arm5,arm6};
}


