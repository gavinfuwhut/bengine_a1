#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "bengine_legged_msgs/MotorPD.h"
#include "bengine_legged_msgs/LowCmd.h"
#include "bengine_legged_msgs/LowState.h"
#include "bengine_legged_msgs/HighState.h"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace bengine_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern bengine_legged_msgs::LowCmd lowCmd;
extern bengine_legged_msgs::LowState lowState;
extern bengine_legged_msgs::MotorPD motorPD;

void motion_init();
void paramInit();
void stand();
void sit();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);
}

#endif
