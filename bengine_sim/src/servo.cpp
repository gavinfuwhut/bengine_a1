#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "bengine_legged_msgs/MotorPD.h"
#include "bengine_legged_msgs/LowCmd.h"
#include "bengine_legged_msgs/LowState.h"
#include "bengine_legged_msgs/MotorCmd.h"
#include "bengine_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "body.h"

using namespace std;
using namespace bengine_model;

bool start_up = true;

class multiThread
{
public:
    multiThread(){
        change_pd = nm.subscribe("/motor_pd" ,1 ,&multiThread::MotorPDCallback, this);
        imu_sub = nm.subscribe("/imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/RF_foot_contact/the_force", 1, &multiThread::RFfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/LF_foot_contact/the_force", 1, &multiThread::LFfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/LB_foot_contact/the_force", 1, &multiThread::LBfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RB_foot_contact/the_force", 1, &multiThread::RBfootCallback, this);
        servo_sub[0] = nm.subscribe("/bengine/RF_hip_controller/state", 1, &multiThread::RFhipCallback, this);
        servo_sub[1] = nm.subscribe("/bengine/RF_thigh_controller/state", 1, &multiThread::RFthighCallback, this);
        servo_sub[2] = nm.subscribe("/bengine/RF_calf_controller/state", 1, &multiThread::RFcalfCallback, this);
        servo_sub[3] = nm.subscribe("/bengine/LF_hip_controller/state", 1, &multiThread::LFhipCallback, this);
        servo_sub[4] = nm.subscribe("/bengine/LF_thigh_controller/state", 1, &multiThread::LFthighCallback, this);
        servo_sub[5] = nm.subscribe("/bengine/LF_calf_controller/state", 1, &multiThread::LFcalfCallback, this);
        servo_sub[6] = nm.subscribe("/bengine/LB_hip_controller/state", 1, &multiThread::LBhipCallback, this);
        servo_sub[7] = nm.subscribe("/bengine/LB_thigh_controller/state", 1, &multiThread::LBthighCallback, this);
        servo_sub[8] = nm.subscribe("/bengine/LB_calf_controller/state", 1, &multiThread::LBcalfCallback, this);
        servo_sub[9] = nm.subscribe("/bengine/RB_hip_controller/state", 1, &multiThread::RBhipCallback, this);
        servo_sub[10] = nm.subscribe("/bengine/RB_thigh_controller/state", 1, &multiThread::RBthighCallback, this);
        servo_sub[11] = nm.subscribe("/bengine/RB_calf_controller/state", 1, &multiThread::RBcalfCallback, this);
        
        
    }

    void MotorPDCallback(const bengine_legged_msgs::MotorPD & msg)
    {
        motorPD.KP = msg.KP;
        motorPD.KD = msg.KD;
        motorPD.gait = msg.gait;
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    {
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;
        
        lowState.imu.acceleration[0] = msg.linear_acceleration.x;
        lowState.imu.acceleration[1] = msg.linear_acceleration.y;
        lowState.imu.acceleration[2] = msg.linear_acceleration.z;
    }

    void RFhipCallback(const bengine_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].position = msg.position;
        lowState.motorState[0].velocity = msg.velocity;
        lowState.motorState[0].torque = msg.torque;
    }

    void RFthighCallback(const bengine_legged_msgs::MotorState& msg)
    {
        lowState.motorState[1].position = msg.position;
        lowState.motorState[1].velocity = msg.velocity;
        lowState.motorState[1].torque = msg.torque;
    }

    void RFcalfCallback(const bengine_legged_msgs::MotorState& msg)
    {
        lowState.motorState[2].position = msg.position;
        lowState.motorState[2].velocity = msg.velocity;
        lowState.motorState[2].torque = msg.torque;
    }

    void LFhipCallback(const bengine_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].position = msg.position;
        lowState.motorState[3].velocity = msg.velocity;
        lowState.motorState[3].torque = msg.torque;
    }

    void LFthighCallback(const bengine_legged_msgs::MotorState& msg)
    {
        lowState.motorState[4].position = msg.position;
        lowState.motorState[4].velocity = msg.velocity;
        lowState.motorState[4].torque = msg.torque;
    }

    void LFcalfCallback(const bengine_legged_msgs::MotorState& msg)
    {
        lowState.motorState[5].position = msg.position;
        lowState.motorState[5].velocity = msg.velocity;
        lowState.motorState[5].torque = msg.torque;
    }

    void LBhipCallback(const bengine_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].position = msg.position;
        lowState.motorState[6].velocity = msg.velocity;
        lowState.motorState[6].torque = msg.torque;
    }

    void LBthighCallback(const bengine_legged_msgs::MotorState& msg)
    {
        lowState.motorState[7].position = msg.position;
        lowState.motorState[7].velocity = msg.velocity;
        lowState.motorState[7].torque = msg.torque;
    }

    void LBcalfCallback(const bengine_legged_msgs::MotorState& msg)
    {
        lowState.motorState[8].position = msg.position;
        lowState.motorState[8].velocity = msg.velocity;
        lowState.motorState[8].torque = msg.torque;
    }

    void RBhipCallback(const bengine_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].position = msg.position;
        lowState.motorState[9].velocity = msg.velocity;
        lowState.motorState[9].torque = msg.torque;
    }

    void RBthighCallback(const bengine_legged_msgs::MotorState& msg)
    {
        lowState.motorState[10].position = msg.position;
        lowState.motorState[10].velocity = msg.velocity;
        lowState.motorState[10].torque = msg.torque;
    }

    void RBcalfCallback(const bengine_legged_msgs::MotorState& msg)
    {
        lowState.motorState[11].position = msg.position;
        lowState.motorState[11].velocity = msg.velocity;
        lowState.motorState[11].torque = msg.torque;
    }

    void RFfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void LFfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void LBfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RBfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }

private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, change_pd;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bengine_gazebo_servo");

    multiThread listen_publish_obj;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
                // ros::Rate loop_rate(1000);
                // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<bengine_legged_msgs::LowState>("/bengine/lowState/state", 1);
    servo_pub[0] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RF_hip_controller/command", 1);
    servo_pub[1] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RF_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RF_calf_controller/command", 1);
    servo_pub[3] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LF_hip_controller/command", 1);
    servo_pub[4] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LF_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LF_calf_controller/command", 1);
    servo_pub[6] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LB_hip_controller/command", 1);
    servo_pub[7] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LB_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/LB_calf_controller/command", 1);
    servo_pub[9] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RB_hip_controller/command", 1);
    servo_pub[10] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RB_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<bengine_legged_msgs::MotorCmd>("/bengine/RB_calf_controller/command", 1);
    

    motion_init();
    motorPD.gait = 1;

    while (ros::ok()){
        if(motorPD.gait==1)
        {
            stand();
            while(ros::ok()&&motorPD.gait==1)
            {
                sendServoCmd();
            }
        }
        if(motorPD.gait==2)
        {
            sit();
            while(ros::ok()&&motorPD.gait==2)
            {
                sendServoCmd();
            }
        }
        // lowState_pub.publish(lowState);
        // sendServoCmd();
    }
    return 0;
}
