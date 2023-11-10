#include "base_controller/hardware_interface.hpp"

#define UART_PATH_FRONT_TMOTOR (char *)"/dev/ttyACM0"
#define UART_PATH_BACK_TMOTOR (char *)"/dev/ttyACM3"
#define UART_PATH_FRONT_ZL (char *)"/dev/ttyACM2"
#define UART_PATH_BACK_ZL (char *)"/dev/ttyACM1"

HardwareInterface::HardwareInterface()
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

    _frontTmotorThread = new LoopFunc("front Tmotor sendrecv thread", 0.002, boost::bind(&uartSendRecvTmotor, _uartFrontTmotor, &_frontModeMsg, &_frontModeState));
    _backTmotorThread = new LoopFunc("back Tmotor sendrecv thread", 0.002, boost::bind(&uartSendRecvTmotor, _uartBackTmotor, &_backModeMsg, &_backModeState));
    _frontZLWheelhubThread = new LoopFunc("front zl wheelhub sendrecv thread", 0.002, boost::bind(&uartSendRecvZLWheelhub, _uartFrontZLWheelhub, &_frontModeMsg, &_frontModeState));
    _backZLWheelhubThread = new LoopFunc("back zl wheelhub sendrecv thread", 0.002, boost::bind(&uartSendRecvZLWheelhub, _uartBackZLWheelhub, &_backModeMsg, &_backModeState));

    _frontTmotorThread->start();
    _backTmotorThread->start();
    _frontZLWheelhubThread->start();
    _backZLWheelhubThread->start();
}

HardwareInterface::~HardwareInterface()
{
    uart_close(_uartFrontTmotor);
    uart_close(_uartBackTmotor);
    uart_close(_uartFrontZLWheelhub);
    uart_close(_uartBackZLWheelhub);

    delete _frontTmotorThread;
    delete _backTmotorThread;
    delete _frontZLWheelhubThread;
    delete _backZLWheelhubThread;
    std::cout << "uart stopped! " << std::endl;
}

void HardwareInterface::_eixtMotor()
{
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

void HardwareInterface::_initDriveTmotor()
{
    enterMotorModeCmdTmotor(&_frontModeMsg.TmotorLeftMsg);
    enterMotorModeCmdTmotor(&_frontModeMsg.TmotorRightMsg);
    uartSendTmotor(_uartFrontTmotor, &_frontModeMsg);

    enterMotorModeCmdTmotor(&_backModeMsg.TmotorLeftMsg);
    enterMotorModeCmdTmotor(&_backModeMsg.TmotorRightMsg);
    uartSendTmotor(_uartBackTmotor, &_backModeMsg);
}

void HardwareInterface::_initDriveZL()
{
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

bool HardwareInterface::sendRecv(RobotCmd *cmd, RobotState *state)
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

    // uartSendRecvTmotor(uartFrontTmotor, &_frontModeMsg, &frontModeState);
    // uartSendRecvTmotor(uartBackTmotor, &backModeMsg, &backModeState);
    // uartSendRecvZLWheelhub(uartFrontZLWheelhub, &_frontModeMsg, &frontModeState);
    //     uartSendRecvZLWheelhub(uartBackZLWheelhub, &backModeMsg, &backModeState);
    // std::cout << "front" << std::endl;
    // showState(_frontModeState);
    // std::cout << "back" << std::endl;
    // showState(_backModeState);

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

void HardwareInterface::getLeftPosition()
{

    float front_left_pos_ = state.q[1];
    float back_left_pos_ = state.q[3];
    std::cout << "front_left_pos_: " << front_left_pos_ << std::endl;
    std::cout << "back_left_pos_: " << back_left_pos_ << std::endl;
}

void HardwareInterface::getRightPosition()
{

    float front_right_pos_ = state.q[7];
    float back_right_pos_ = state.q[5];
    std::cout << "front_right_pos_: " << front_right_pos_ << std::endl;
    std::cout << "back_right_pos_: " << back_right_pos_ << std::endl;
    std::cout << "             " << std::endl;
}

void HardwareInterface::getLeftVelocity()
{

    float front_left_vel_ = state.qd[1] * M_PI * 0.14 /60;
    float back_left_vel_ = state.qd[3] * M_PI * 0.14 /60;
    std::cout << "front_left_vel_: " << front_left_vel_ << std::endl;
    std::cout << "back_left_vel_: " << back_left_vel_ << std::endl;
}

void HardwareInterface::getRightVelocity()
{
    float front_right_vel_ = state.qd[7] * M_PI * 0.14 /60;
    float back_right_vel_ = state.qd[5] * M_PI * 0.14 /60;
    std::cout << "front_right_vel_: " << front_right_vel_ << std::endl;
    std::cout << "back_right_vel_: " << back_right_vel_ << std::endl;
}
void HardwareInterface::respectiveCommand(float front_left_vel, float front_right_vel, float back_left_vel, float back_right_vel,
                                          float front_left_ang, float front_right_ang, float back_left_ang, float back_right_ang)
{
    // std::cout<<"front_left_vel"<< front_left_vel<<std::endl;
    // std::cout<<"front_right_vel"<< front_right_vel<<std::endl;

    double _maxTurningSpeed = 1;
    cmd.qd[1] = front_left_vel   * 60 / 3.14 /0.14;
    cmd.qd[7] = front_right_vel  * 60 / 3.14 /0.14;
    cmd.qd[3] = -back_left_vel * 60 / 3.14 /0.14;
    cmd.qd[5] = back_right_vel  * 60 / 3.14 /0.14;

    // 转向电机的位置比例、微分系数
    double kp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double kd[8] = {5, 5, 5, 5, 5, 5, 5, 5};
    //  double kd[8] = {.1, 5, .1, 5, .1, 5, .1, 5};
    for (int i = 0; i < 8; i++)
    {
        cmd.kp(i) = kp[i];
        cmd.kd(i) = kd[i];
    }

    // double wheelSpeed = angular * _maxTurningSpeed;
    // double _direction = -M_PI_4;
    double _direction = 0;
    double _maxDirectionChangeRate = 0.03;
    double _maxYawAngle = 0.785;

    // for(int i(0); i<4; i++)
    // {
    //     cmd.q[2*i] = _direction * _maxYawAngle;
    // }
    // std::cout<<"debug"<<cmd.q[0]<<std::endl;

    // cmd.q[0] = front_left_ang;
    // cmd.q[6] = front_right_ang;
    // cmd.q[2] = back_left_ang;
    // cmd.q[4] = back_right_ang;

    // cmd.qd[0] = front_left_ang * _maxTurningSpeed;
    // cmd.qd[6] = front_right_ang * _maxTurningSpeed ;
    // cmd.qd[2] = back_left_ang * _maxTurningSpeed ;
    // cmd.qd[4] = back_right_ang * _maxTurningSpeed;
    // std::cout << "weizhi" << state.q[0] << std::endl;
    // std::cout<<"sudo dianji"<<cmd.qd[0]<<std::endl;
   

    // std::cout << "sudu" << state.qd[0] << std::endl;
    // std::cout << "weizhi" << state.q[6] << std::endl;
    // std::cout << "sudu" << state.qd[6] << std::endl;
    // std::cout << "weizhi" << state.q[2] << std::endl;
    // std::cout << "sudu" << state.qd[2] << std::endl;
    // std::cout << "weizhi" << state.q[4] << std::endl;
    // std::cout << "sudu" << state.qd[4] << std::endl;
    
    // if(abs(front_left_ang - state.q[0]) > M_PI_2 && front_left_ang != 0)
    // {
    //     front_left_ang = front_left_ang - (front_left_ang - state.q[0]) / abs(front_left_ang - state.q[0]) *M_PI;

    // }

    // for(int i(0); i<ROBOT_ACTUATOR_NUM/2; ++i)
    // {
    //     if((cmd.q[2*i] - state.q[2*i] > _maxDirectionChangeRate) || (cmd.q[2*i] - state.q[2*i] < -_maxDirectionChangeRate)){
    //         cmd.q[2*i] = abs(cmd.q[2*i] - state.q[2*i]) / (cmd.q[2*i] - state.q[2*i]) * _maxDirectionChangeRate + state.q[2*i];
    //     }
    // }10
    sendRecv(&cmd, &state);
}

// void respectiveAngle()
// {

// }
