/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"

namespace unitree_model {

/*
ros::Publisher servo_pub[12];
unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    // for(int i=0; i<4; i++){
    //     lowCmd.motorCmd[i*3+0].mode = 0x0A;
    //     lowCmd.motorCmd[i*3+0].Kp = 70;
    //     lowCmd.motorCmd[i*3+0].dq = 0;
    //     lowCmd.motorCmd[i*3+0].Kd = 3;
    //     lowCmd.motorCmd[i*3+0].tau = 0;
    //     lowCmd.motorCmd[i*3+1].mode = 0x0A;
    //     lowCmd.motorCmd[i*3+1].Kp = 180;
    //     lowCmd.motorCmd[i*3+1].dq = 0;
    //     lowCmd.motorCmd[i*3+1].Kd = 8;
    //     lowCmd.motorCmd[i*3+1].tau = 0;
    //     lowCmd.motorCmd[i*3+2].mode = 0x0A;
    //     lowCmd.motorCmd[i*3+2].Kp = 300;
    //     lowCmd.motorCmd[i*3+2].dq = 0;
    //     lowCmd.motorCmd[i*3+2].Kd = 15;
    //     lowCmd.motorCmd[i*3+2].tau = 0;
    // }
    // for(int i=0; i<12; i++){
    //     lowCmd.motorCmd[i].q = lowState.motorState[i].q;
    // }
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 0;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 0;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 0;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 0;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 0;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 0;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
}

void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 2*1000);
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    // usleep(1000);
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
        usleep(500);
    }
}

void motion_reset()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 70;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 8;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }

    // double pos[12] = {-0.0, 0.5, -1.4, -0.0, 0.5, -1.4, 
    //                   0.0, 0.5, -1.4, -0.0, 0.5, -1.4};
    // double pos[12] = {-0.0, 0.78, -1.57, -0.0, 0.78, -1.57, 
    //                   0.0, 0.78, -1.57, -0.0, 0.78, -1.57};
    // double pos[12] = {-0.0, 0.65, -1.57, -0.0, 0.65, -1.57, 
    //                   0.0, 0.65, -1.57, -0.0, 0.65, -1.57};
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 5000);

    sendServoCmd();
}

void param_reset()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 0;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 0;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 0;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 0;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 0;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 0;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
}
*/



ros::Publisher servo_pub[12];
unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 0;//70;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 0;//3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 0;//180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 0;//8;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 0;//300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 0;//15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
}

void stand()
{   
    double pos[12] = {0.0, 0.5, -1.4, -0.0, 0.5, -1.4, 
                      0.0, 0.5, -1.4, -0.0, 0.5, -1.4};
    moveAllPosition(pos, 2000);
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd()
{
    
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
        // ROS_INFO("Publishing command to motor %lu: q=%f, dq=%f, tau=%f, Kp=%f, Kd=%f", m, lowCmd.motorCmd[m].q, lowCmd.motorCmd[m].dq, lowCmd.motorCmd[m].tau, lowCmd.motorCmd[m].Kp, lowCmd.motorCmd[m].Kd);
    
    }
    ros::spinOnce();
    //usleep(1000); // in microseconds
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
        usleep(500);
    }
}

void motion_reset()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 70;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 8;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }

    // double pos[12] = {-0.0, 0.5, -1.4, -0.0, 0.5, -1.4, 
    //                   0.0, 0.5, -1.4, -0.0, 0.5, -1.4};
    double pos[12] = {-0.0, 0.785, -1.57, -0.0, 0.785, -1.57, 
                      0.0, 0.785, -1.57, -0.0, 0.785, -1.57};
    // double pos[12] = {-0.0, 0.65, -1.57, -0.0, 0.65, -1.57, 
    //                   0.0, 0.65, -1.57, -0.0, 0.65, -1.57};
    moveAllPosition(pos, 2000);

    sendServoCmd();
}

void motion_reset(double* targetPos, double duration)
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 70;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 8;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }

    moveAllPosition(targetPos, duration);

    sendServoCmd();
}

void param_reset()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 0;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 0;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 0;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 0;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 0;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 0;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
}

}
