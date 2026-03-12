#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

void RosShutDown(int sig){
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    // start subscriber
    initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();   

    signal(SIGINT, RosShutDown);
    // std::cout << "The control interfac" << std::endl;

    cmdPanel = new KeyBoard();
}

IOROS::~IOROS(){
    delete cmdPanel;
    ros::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].position = lowCmd->motorCmd[i].position;
        _lowCmd.motorCmd[i].velocity  = lowCmd->motorCmd[i].velocity;
        _lowCmd.motorCmd[i].torque = lowCmd->motorCmd[i].torque;
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
    }
    for(int m(0); m < 12; ++m){
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
}

void IOROS::recvState(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].position = _lowState.motorState[i].position;
        state->motorState[i].velocity = _lowState.motorState[i].velocity;
        state->motorState[i].torque = _lowState.motorState[i].torque;
    }
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.acceleration[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS::initSend(){
    _servo_pub[0] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RF_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RF_thigh_controller/command", 1);
    _servo_pub[2] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RF_calf_controller/command", 1);
    _servo_pub[3] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LF_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LF_thigh_controller/command", 1);
    _servo_pub[5] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LF_calf_controller/command", 1);
    _servo_pub[6] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LB_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LB_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LB_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RB_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RB_thigh_controller/command", 1);
    _servo_pub[11] = _nm.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RB_calf_controller/command", 1);
}

void IOROS::initRecv(){
    _imu_sub = _nm.subscribe("/imu", 1, &IOROS::imuCallback, this);
    _servo_sub[0] = _nm.subscribe("/bengine/RF_hip_controller/state", 1, &IOROS::RFhipCallback, this);
    _servo_sub[1] = _nm.subscribe("/bengine/RF_thigh_controller/state", 1, &IOROS::RFthighCallback, this);
    _servo_sub[2] = _nm.subscribe("/bengine/RF_calf_controller/state", 1, &IOROS::RFcalfCallback, this);
    _servo_sub[3] = _nm.subscribe("/bengine/LF_hip_controller/state", 1, &IOROS::LFhipCallback, this);
    _servo_sub[4] = _nm.subscribe("/bengine/LF_thigh_controller/state", 1, &IOROS::LFthighCallback, this);
    _servo_sub[5] = _nm.subscribe("/bengine/LF_calf_controller/state", 1, &IOROS::LFcalfCallback, this);
    _servo_sub[6] = _nm.subscribe("/bengine/LB_hip_controller/state", 1, &IOROS::LBhipCallback, this);
    _servo_sub[7] = _nm.subscribe("/bengine/LB_thigh_controller/state", 1, &IOROS::LBthighCallback, this);
    _servo_sub[8] = _nm.subscribe("/bengine/LB_calf_controller/state", 1, &IOROS::LBcalfCallback, this);
    _servo_sub[9] = _nm.subscribe("/bengine/RB_hip_controller/state", 1, &IOROS::RBhipCallback, this);
    _servo_sub[10] = _nm.subscribe("/bengine/RB_thigh_controller/state", 1, &IOROS::RBthighCallback, this);
    _servo_sub[11] = _nm.subscribe("/bengine/RB_calf_controller/state", 1, &IOROS::RBcalfCallback, this);

}

void IOROS::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    // std::cout << "msg.angular_velocity.x=" <<msg.angular_velocity.x << std::endl;
    
    _lowState.imu.acceleration[0] = msg.linear_acceleration.x;
    _lowState.imu.acceleration[1] = msg.linear_acceleration.y;
    _lowState.imu.acceleration[2] = msg.linear_acceleration.z;
}

void IOROS::RFhipCallback(const bengine_legged_msgs::MotorState& msg)
{
    _lowState.motorState[0].position = msg.position;
    _lowState.motorState[0].velocity = msg.velocity;
    _lowState.motorState[0].torque = msg.torque;
}

void IOROS::RFthighCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[1].position = msg.position;
    _lowState.motorState[1].velocity = msg.velocity;
    _lowState.motorState[1].velocity = msg.velocity;
}

void IOROS::RFcalfCallback(const bengine_legged_msgs::MotorState& msg)
{
    _lowState.motorState[2].position = msg.position;
    _lowState.motorState[2].velocity = msg.velocity;
    _lowState.motorState[2].velocity = msg.velocity;
    // std::cout << "motorState[2].position=" <<_lowState.motorState[2].position << std::endl;
}

void IOROS::LFhipCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[3].position = msg.position;
    _lowState.motorState[3].velocity = msg.velocity;
    _lowState.motorState[3].velocity = msg.velocity;
}

void IOROS::LFthighCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[4].position = msg.position;
    _lowState.motorState[4].velocity = msg.velocity;
    _lowState.motorState[4].velocity = msg.velocity;
}

void IOROS::LFcalfCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[5].position = msg.position;
    _lowState.motorState[5].velocity = msg.velocity;
    _lowState.motorState[5].velocity = msg.velocity;
}

void IOROS::LBhipCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[6].position = msg.position;
    _lowState.motorState[6].velocity = msg.velocity;
    _lowState.motorState[6].velocity = msg.velocity;
}

void IOROS::LBthighCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[7].position = msg.position;
    _lowState.motorState[7].velocity = msg.velocity;
    _lowState.motorState[7].velocity = msg.velocity;
}

void IOROS::LBcalfCallback(const bengine_legged_msgs::MotorState& msg)
{
    _lowState.motorState[8].position = msg.position;
    _lowState.motorState[8].velocity = msg.velocity;
    _lowState.motorState[8].velocity = msg.velocity;
}

void IOROS::RBhipCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[9].position = msg.position;
    _lowState.motorState[9].velocity = msg.velocity;
    _lowState.motorState[9].velocity = msg.velocity;
}

void IOROS::RBthighCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[10].position = msg.position;
    _lowState.motorState[10].velocity = msg.velocity;
    _lowState.motorState[10].velocity = msg.velocity;
}

void IOROS::RBcalfCallback(const bengine_legged_msgs::MotorState& msg)
{

    _lowState.motorState[11].position = msg.position;
    _lowState.motorState[11].velocity = msg.velocity;
    _lowState.motorState[11].velocity = msg.velocity;
}
