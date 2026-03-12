#include "body.h"

namespace bengine_model {

ros::Publisher servo_pub[12];
bengine_legged_msgs::LowCmd lowCmd;
bengine_legged_msgs::LowState lowState;
bengine_legged_msgs::MotorPD motorPD;

void motion_init()
{
    paramInit();
}
// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        
        lowCmd.motorCmd[i*3+0].Kp = 100;
        lowCmd.motorCmd[i*3+0].velocity = 0;
        lowCmd.motorCmd[i*3+0].Kd = 5;
        lowCmd.motorCmd[i*3+0].torque = 0;
        
        lowCmd.motorCmd[i*3+1].Kp = 135;
        lowCmd.motorCmd[i*3+1].velocity = 0;
        lowCmd.motorCmd[i*3+1].Kd = 15;
        lowCmd.motorCmd[i*3+1].torque = 0;
       
        lowCmd.motorCmd[i*3+2].Kp = 150;
        lowCmd.motorCmd[i*3+2].velocity = 0;
        lowCmd.motorCmd[i*3+2].Kd = 20;
        lowCmd.motorCmd[i*3+2].torque = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].position = lowState.motorState[i].position;
    }
}

void stand()
{   
    double pos[12] = {0.0, -0.26, 1.0, -0.0, 0.26, -1.0, 
                      -0.0, 0.26, -1.0, 0.0, -0.26, 1.0};
    moveAllPosition(pos, 2*1000);
}

void sit()
{
    double pos[12] = {0.0, -0.01, 0.1, -0.0, 0.01, -0.1, 
                      -0.0, 0.01, -0.1, 0.0, -0.01, 0.1};
    moveAllPosition(pos, 4*1000);
}

void sendServoCmd()
{
    if(motorPD.KP[0]>0)
    {
        for(int i=0; i<4; i++){
            
            lowCmd.motorCmd[i*3+0].Kp = motorPD.KP[i*3+0];
            lowCmd.motorCmd[i*3+0].velocity = 0;
            lowCmd.motorCmd[i*3+0].Kd = motorPD.KD[i*3+0];
            lowCmd.motorCmd[i*3+0].torque = 0;
            
            lowCmd.motorCmd[i*3+1].Kp = motorPD.KP[i*3+1];
            lowCmd.motorCmd[i*3+1].velocity = 0;
            lowCmd.motorCmd[i*3+1].Kd = motorPD.KD[i*3+1];
            lowCmd.motorCmd[i*3+1].torque = 0;
        
            lowCmd.motorCmd[i*3+2].Kp = motorPD.KP[i*3+2];
            lowCmd.motorCmd[i*3+2].velocity = 0;
            lowCmd.motorCmd[i*3+2].Kd = motorPD.KD[i*3+2];
            lowCmd.motorCmd[i*3+2].torque = 0;
        }
    }
        
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    usleep(1000);
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].position;
    for(int i=1; i<=duration; i++)
    {
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].position = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
    }
}


}
