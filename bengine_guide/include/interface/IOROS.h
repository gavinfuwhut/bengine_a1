#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "bengine_legged_msgs/LowCmd.h"
#include "bengine_legged_msgs/LowState.h"
#include "bengine_legged_msgs/MotorCmd.h"
#include "bengine_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <string>

class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);
ros::NodeHandle _nm;
ros::Subscriber _servo_sub[12], _imu_sub;
ros::Publisher _servo_pub[12];
bengine_legged_msgs::LowCmd _lowCmd;
bengine_legged_msgs::LowState _lowState;

//repeated functions for multi-thread
void initRecv();
void initSend();

//Callback functions for ROS
void imuCallback(const sensor_msgs::Imu & msg);

void RFhipCallback(const bengine_legged_msgs::MotorState& msg);
void RFthighCallback(const bengine_legged_msgs::MotorState& msg);
void RFcalfCallback(const bengine_legged_msgs::MotorState& msg);

void LFhipCallback(const bengine_legged_msgs::MotorState& msg);
void LFthighCallback(const bengine_legged_msgs::MotorState& msg);
void LFcalfCallback(const bengine_legged_msgs::MotorState& msg);

void LBhipCallback(const bengine_legged_msgs::MotorState& msg);
void LBthighCallback(const bengine_legged_msgs::MotorState& msg);
void LBcalfCallback(const bengine_legged_msgs::MotorState& msg);

void RBhipCallback(const bengine_legged_msgs::MotorState& msg);
void RBthighCallback(const bengine_legged_msgs::MotorState& msg);
void RBcalfCallback(const bengine_legged_msgs::MotorState& msg);
};

#endif  // IOROS_H
