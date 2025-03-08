/*****************************************************************
 Test CPG with LCM on hardware

 NOTE: The following are NOT allowed without explicit permission 
    from Guillaume or Milad:
        - NO torque control (i.e. any changes to tauDes)
        - NO power limit increases (i.e. in res1)
        - NO large gains (i.e. larger than Kp 60, Kd 2)
        - anything that did not work in Gazebo
        - anything near the torque or velocity limits of the robot (MUST CHECK IN GAZEBO)
    
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

#include "convert.h"
#include "ros/ros.h"
#include "body.h" 
#include "../include/CurrentState.h"
#include "../../include/OrientationEstimator.h"
#include "../../include/PositionVelocityEstimator.h"
#include "../../include/ContactEstimator.h"
#include "../../include/LegController.h"
#include "../../include/ControlFSMData.h"
#include "../../CPG/HopfPolar.h"
#include "../../include/Math/MathUtilities.h"

#include <chrono>

using namespace std;
using namespace UNITREE_LEGGED_SDK;
using namespace unitree_model;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Go1), 
                            udp(level), 
                            t(traj_len),
                            mf_traj(traj_len),
                            me_traj(traj_len),
                            listen_publish_obj("go1") {
        udp.InitCmdData(cmd);
    }
    void init();
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void printState();
    void fixFootForce();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0}; // from robot with LCM, convert to lowState ros msg as in rest of repo
    unitree_legged_msgs::LowState lowStateCopy; // used to save cheat data coming from T265

    // commands to send motors
    float qDes[12]={0};
    float dqDes[12] = {0};
    float tauDes[12] = {0};
    float Kp[12] = {0};  
    float Kd[12] = {0};

    int rate_count = 0;
    int motiontime = 0;
    float dt = 0.001;     // 0.001~0.01, originally 0.002
    std::chrono::steady_clock::time_point start, end;
    int time_vector[2] = { 5, 15 }; // times to switch modes in seconds
    int muscle_time[3] = { 8, 10, 12};

    float footForceZero[4] = {1000.0, 1000.0, 1000.0, 1000.0}; // fix "zero" value, sensors read 100+ N even when in air
    // float footForcePrev[4] = {0.0}; // fix "zero" value, sensors read 100+ N even when in air
    double init_joint_pos[12] = {0};
    // double default_joint_pos[12] = { 0., 0.785, -1.57,
    //                                0., 0.785, -1.57,
    //                                0., 0.785, -1.57,
    //                                0., 0.785, -1.57  };
    // double default_joint_pos[12] = { 0., 0.9, -1.57,
    //                                0., 0.9, -1.57,
    //                                0., 0.9, -1.57,
    //                                0., 0.9, -1.57  };
    double default_joint_pos[12] = { 0., 0.95, -1.7,
                                   0., 0.95, -1.7,
                                   0., 0.95, -1.7,
                                   0., 0.95, -1.7  };
    double current_dof_pos[12] = {0};
    double current_dof_vel[12] = {0};
    int hips[4] = { 0, 3, 6, 9 };

    // Ekeberg
    int NUM_SECONDS = time_vector[1] - time_vector[0];
    int traj_len = NUM_SECONDS*1000 + 1;
    std::vector<double> t; //(traj_len);
    // muscle activation trajectories
    std::vector<double> me_traj;//(traj_len);
    std::vector<double> mf_traj;//(traj_len);
    
    double ME = 1;  //  max muscle extension
    double MF = 1;  //  max muscle flexion
    double f = 0.8; //  frequency


    // CPG 
    HopfPolar cpg;
    Vec4<double> x_out, z_out;
    double sideSign[4] = {-1, 1, -1, 1};
    double foot_y = 0.08;

    // all quadruped objects
    Quadruped quad;
    StateEstimate stateEstimate;
    LegController* legController;
    StateEstimatorContainer* stateEstimator;
    ControlFSMData* _controlData;
    CurrentState listen_publish_obj;

    // ros
    ros::NodeHandle n;
};

void Custom::init()
{  
    std::cout << "set quadruped" << std::endl;
    quad.setQuadruped(1); // 1 for Go1, 2 for A1

    // initialize new leg controller and state estimate object
    std::cout << "start Leg Controller" << std::endl;
    legController = new LegController(quad);
    legController->zeroCommand();

    std::cout << "start state estimate" << std::endl;
    stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    // using IMU sensors 
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();

    // initialize FSMData
    std::cout << "start controlFSMData" << std::endl;
    _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;

    cpg.init();
    std::cout << "CPG started" << std::endl;

    // init Ekeberg 

    // design muscle profiles (see Legged Robots course slides)
    for (int i=0; i<traj_len; i++){
        t[i] = dt*i;
        me_traj[i] = ME * 0.5 * ( std::sin(2*M_PI * f * t[i]) + 1);
        mf_traj[i] = 1 - MF * 0.5 * ( std::sin(2*M_PI * f * t[i]) + 1);
    }
    std::cout << "Ekeberg trajectories init" << std::endl;

    start = std::chrono::steady_clock::now();
    
}


void Custom::UDPRecv()
{  
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}


void Custom::printState(){
    std::cout << "====================\nstate: \n===================="  << std::endl;
    std::cout << "levelFlag " << state.levelFlag << std::endl;
    std::cout << "commVersion " << state.version[0] << std::endl;
    std::cout << "robotID " << state.version[1] << std::endl;
    std::cout << "SN " << state.SN[0] << std::endl;
    std::cout << "bandWidth " << state.bandWidth << std::endl;
    std::cout << "tick " << state.tick << std::endl;

}

/*
 *  Fix foot force, gravity compensated somewhere? 
 */
void Custom::fixFootForce(){
    for (int i=0; i< 4; i++){
        // check if we should update the Zero value
        if (lowState.footForce[i] != 0) // may not be initialized yet 
            footForceZero[i] = fmax( fmin(footForceZero[i], lowState.footForce[i]), 0.);
        // subtract Zero value from measurement, scale (gravity compensation?)
        lowState.footForce[i] = (lowState.footForce[i] - footForceZero[i]) / 9.8; // is this more reasonable? 
        // lowState.footForce[i] = lowState.footForce[i] / 10; //9.8;
    }
    // std::cout << "footForceZero "<< footForceZero[0] << " "
    //                              << footForceZero[1] << " "
    //                              << footForceZero[2] << " "
    //                              << footForceZero[3] << " " << std::endl;
    // std::cout << "footForce "    << lowState.footForce[0] << " "
    //                              << lowState.footForce[1] << " "
    //                              << lowState.footForce[2] << " "
    //                              << lowState.footForce[3] << " " << std::endl;
}


void Custom::RobotControl() 
{
    
    motiontime++;
    udp.GetRecv(state);
    // std::cout << "state " << state.footForce[0] << std::endl;
    // printState();
    lowStateCopy = lowState; // save temporary lowState, cheat data is coming from elsewhere and gets overwritten next
    lowState = ToRos(state); // lowState as "usual"
    // any data changing that must happen
    lowState.cheat = lowStateCopy.cheat;
    fixFootForce();
    lowState.cheat.position[2] = 0.3; // just set z to desired height from CPG
    // update data, run state estimator
    _controlData->_legController->updateData();
    _controlData->_stateEstimator->run();
    // std::cout << lowState << std::endl;

    // printf("%d  %f\n", motiontime, lowState.motorState[FR_2].q);
    // printf("lowState   %d  %f\n", motiontime, lowState.imu.quaternion[2]); 
    // printf("RecvLowROS %d  %f\n", motiontime, RecvLowROS.imu.quaternion[2]);

    auto& seResult = _controlData->_stateEstimator->getResult();
    for (int i =0; i<4; i++){
        for (int j = 0; j < 3; j++){
            current_dof_pos[3*i+j] =  _controlData->_legController->data[i].q[j];
            current_dof_vel[3*i+j] =  _controlData->_legController->data[i].qd[j];
        }     
    }

    if( motiontime >= 0){
        // first, record initial position
        if( motiontime >= 0 && motiontime < 10){
            for (int i=0; i < 12; i++){
                init_joint_pos[i] = lowState.motorState[i].q;
            }
        }
        // second, move to the default joint position with Kp Kd
        if( motiontime >= 10 && motiontime < 3000){
            rate_count++;
            double rate = rate_count/2990.0;  // needs count to 200
            for (int i=0; i<12; i++){
                Kp[i] = 55; //5.0;
                Kd[i] = 1.5;
                qDes[i] = jointLinearInterpolation(init_joint_pos[i], default_joint_pos[i], rate);
            }
        }

        // third, update the CPG and set joint commands
        if( motiontime >= time_vector[0]*1000){


            // test muscle model
            double alpha, beta, gamma, delta;
            alpha=1;//20;
            beta=1;//20;
            gamma=5;
            delta=0.1;

            alpha = 0; beta = 10; gamma = 2; delta = 1; // video 1
            alpha = 0; beta = 10; gamma = 1; delta = 1; // video 2
            alpha = 0; beta = 10; gamma = 3; delta = 1; // video 3 (stays where it is.)

            alpha = 0; beta = 10; gamma = 2; delta = 1; // video 4
            alpha = 0; beta = 10; gamma = 1; delta = 1; // video 5
            alpha = 0; beta = 10; gamma = 3; delta = 1; // video 6
            alpha = 0; beta = 10; gamma = 1.5; delta = 1; // video 7
            alpha = 3; beta = 10; gamma = 2; delta = 1; // video 8

            alpha = 3; beta = 10; gamma = 1.5; delta = 1; // video 8

            if (motiontime < muscle_time[0]*1000){
                alpha = 3; beta = 10; gamma = 1.5; delta = 1;
            }    
            else if (motiontime < muscle_time[1]*1000) {
                alpha = 3; beta = 10; gamma = 0.5; delta = 1;
            }
            else if (motiontime < muscle_time[2]*1000) {
                alpha = 3; beta = 10; gamma = 1.5; delta = 1;
            }
            else {
                alpha = 3; beta = 10; gamma = 4; delta = 1;
            }


            double ML[12] = {0.};
            double MR[12] = {0.};
            //ML[1] = std::sin(motiontime * 2 * 3.14); //0; 
            
            
            // std::cout << "tauDes "; 
            for (int i=0; i<6; i++){
                // if ((i % 3) + 1 ){
                //     ML[i] = mf_traj[motiontime - time_vector[0]*1000];
                //     MR[i] = me_traj[motiontime - time_vector[0]*1000];
                // }
                // else{
                //     ML[i] = me_traj[motiontime - time_vector[0]*1000];
                //     MR[i] = mf_traj[motiontime - time_vector[0]*1000];
                // }

                // ML[i] = me_traj[motiontime - time_vector[0]*1000];
                // MR[i] = mf_traj[motiontime - time_vector[0]*1000];

                //qDes[i] = PosStopF; // won't affect due to gains at 0
                //dqDes[i] = 0 ;//VelStopF;
                // cmd.motorCmd[i].dq = 0;
                // tauDes[i] = clip(alpha * (ML[i] - MR[i]) 
                //                 + beta * (ML[i] + MR[i] + gamma) * (default_joint_pos[i] - current_dof_pos[i] ) 
                //                 + delta * (-current_dof_vel[i]), 
                //                 -10.0, 10.0);
                tauDes[i] = clip(alpha * (ML[i] - MR[i]), -5.0, 5.0); 
                Kp[i] =  beta * (ML[i] + MR[i] + gamma);
                Kd[i] = delta; //1;

                qDes[i] = default_joint_pos[i];
                dqDes[i] = 0 ;

                // std::cout << " " << i << " " <<  tauDes[i];
                // tauDes[i] = 0;
            }

            for (int i=6; i<12; i++){
                Kp[i] =  1.2*beta * (ML[i] + MR[i] + gamma);
            }

            // keep hips at 0 (nominal)
            for (int i=0; i<4; i++){
                tauDes[hips[i]] = 0; 
                Kp[hips[i]] = 55;
                Kd[hips[i]] = 1; //1;
                qDes[hips[i]] = 0;
                dqDes[hips[i]] = 0 ;
            }
            

            // std::cout << "\n";
            
        }

        // stop after x seconds 
        if (motiontime >= time_vector[1]*1000){
            for (int i=0; i < 12; i++){
                qDes[i] = lowState.motorState[i].q;
                dqDes[i] = 0;
                tauDes[i] = 0;
                Kp[i] = 20.;
                Kd[i] = 2.5;
            }
        }

        // set command
        for (int i=0; i < 12; i++){
            cmd.motorCmd[i].q = qDes[i];
            cmd.motorCmd[i].dq = dqDes[i];
            cmd.motorCmd[i].Kp = Kp[i];
            cmd.motorCmd[i].Kd = Kd[i];
            cmd.motorCmd[i].tau = tauDes[i];
            // cmd.motorCmd[i].mode = 0x0A;
        }

    }


    if(motiontime > 10){
        // EXTREMELY IMPORTANT! NO CHANGES ALLOWED WITHOUT EXPLICIT PERMISSION FROM GUILLAUME OR MILAD
        safe.PositionLimit(cmd);
        int res1 = safe.PowerProtect(cmd, state, 4); // % of power allowed to use [above 5 NEVER ALLOWED without explicit permission]
        // You can uncomment it for position protection
        // int res2 = safe.PositionProtect(cmd, state, 0.087);
        if(res1 < 0) exit(-1);
    }

    udp.SetSend(cmd);
    ros::spinOnce();

}


int main(int argc, char **argv)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    ros::init(argc, argv, "go1_cpg_lcm");

    Custom custom(LOWLEVEL);
    custom.init();

    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
