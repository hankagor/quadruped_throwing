/*****************************************************************
 Test SMALL jump with LCM on hardware

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
#include "../../CPG/JumpingObj.h"

#include <chrono>

using namespace std;
using namespace UNITREE_LEGGED_SDK;
using namespace unitree_model;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Go1), 
                            udp(level), 
                            full_state(full_state_len),
                            full_traj("/data/bellegar/quadruped_logs/full_state_hw_jump.txt"),
                            listen_publish_obj("go1") {
        udp.InitCmdData(cmd);
    }
    void init();
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void printState();
    void fixFootForce();
    void writeOutState();

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
    // int traj_counter = 0; // index through trajectory more easily 
    float dt = 0.001;     // 0.001~0.01, originally 0.002
    // std::chrono::steady_clock::time_point start, end;
    int time_vector[2] = { 4, 9 }; // times to switch modes in seconds
    // int last_time = 12;

    float footForceZero[4] = {1000.0, 1000.0, 1000.0, 1000.0}; // fix "zero" value, sensors read 100+ N even when in air
    double real_foot_forces[4] = {0};
    // float footForcePrev[4] = {0.0}; // fix "zero" value, sensors read 100+ N even when in air
    double init_joint_pos[12] = {0};
    // double default_joint_pos[12] = { 0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3  };
    JumpingObj jumpObj;

    // double default_joint_pos[12] = {0.0, thigh, calf, -0.0, thigh, calf, 
    //                                 0.0, thigh, calf, -0.0, thigh, calf};

    // double default_floor_pos[12] = {-0.0, 1.6, -2.7, -0.0, 1.6, -2.7, 
    //                                  0.0, 1.6, -2.7, -0.0, 1.6, -2.7};

    // time vector 
    // int NUM_SECONDS = time_vector[1] - time_vector[0];

    // write out
    int full_state_len = 76;
    int cpg_state_len = 3;
    std::ofstream full_traj;
    std::vector<double> full_state; //(full_state_len);

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
    if (0){
        // onboard IMU 
        stateEstimator->addEstimator<VectorNavOrientationEstimator>();
        stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();
    }
    else{
        // T265, placing in "cheat" msg
        stateEstimator->addEstimator<CheaterOrientationEstimator>(); 
        stateEstimator->addEstimator<T265PositionVelocityEstimator>();
    }

    // initialize FSMData
    std::cout << "start controlFSMData" << std::endl;
    _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;


    // init jumping object
    // jumpObj.NUM_SECONDS = NUM_SECONDS;
    jumpObj._controlData = _controlData;
    // jumpObj.init();
    time_vector[1] =  time_vector[0] + jumpObj.NUM_SECONDS; // + 1; 

    jumpObj.initJumpingParams();
    jumpObj.init();
    jumpObj.resetObjectiveParams();
    jumpObj.hardware_flag = true;
    if (jumpObj.do_BO){
        jumpObj.updateParams();
    }

    jumpObj.writeOutForceTrajectories("/data/bellegar/quadruped_logs/force_hw_trajectories.txt");
    std::cout << "JumpingObj started" << std::endl;

    // start = std::chrono::steady_clock::now();
    
}

void Custom::writeOutState(){
    jumpObj.getFullState(full_state);
    for(int j=0; j<full_state_len; j++)
        full_traj << full_state[j] << " ";
    full_traj << "\n";
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
    // std::cout << "footForce PRE-CLIP  "    << lowState.footForce[0] << " "
    //                              << lowState.footForce[1] << " "
    //                              << lowState.footForce[2] << " "
    //                              << lowState.footForce[3] << " " << std::endl;
    for (int i=0; i< 4; i++){
        // check if we should update the Zero value
        if (lowState.footForce[i] != 0) // may not be initialized yet 
            footForceZero[i] = fmax( fmin(footForceZero[i], lowState.footForce[i]), 0.);
        // subtract Zero value from measurement, scale (gravity compensation?)
        real_foot_forces[i] = (lowState.footForce[i] - footForceZero[i]) / 5 ;
        lowState.footForce[i] = (lowState.footForce[i] - footForceZero[i]) / 5 ; /// 2.0; // / 9.8; // is this more reasonable? 
        
        // lowState.footForce[i] = lowState.footForce[i] / 10; //9.8;
        jumpObj.real_foot_forces[i] = real_foot_forces[i];
    }
    
    // std::cout << "footForceZero "<< footForceZero[0] << " "
    //                              << footForceZero[1] << " "
    //                              << footForceZero[2] << " "
    //                              << footForceZero[3] << " " << std::endl;
    // std::cout << "footForce POST-CLIP "    << lowState.footForce[0] << " "
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
    // lowState.cheat.position[2] = 0.3; // just set z to desired height from CPG
    // update data, run state estimator
    jumpObj.runStateEstimation();

    // _controlData->_legController->updateData();
    // _controlData->_stateEstimator->run();
    // std::cout << lowState << std::endl;

    // printf("%d  %f\n", motiontime, lowState.motorState[FR_2].q);
    // printf("lowState   %d  %f\n", motiontime, lowState.imu.quaternion[2]); 
    // printf("RecvLowROS %d  %f\n", motiontime, RecvLowROS.imu.quaternion[2]);

    // auto& seResult = _controlData->_stateEstimator->getResult();

    if( motiontime >= 0){
        // first, record initial position
        if( motiontime >= 0 && motiontime < 10){
            for (int i=0; i < 12; i++){
                init_joint_pos[i] = lowState.motorState[i].q;
            }
        }
        // second, move to the default joint position with Kp Kd
        if( motiontime >= 10 && motiontime < 3000 ) { //time_vector[0]*1000){
            rate_count++;
            double rate = rate_count/2990.0;  // needs count to 200
            for (int i=0; i<12; i++){
                Kp[i] = 55; //5.0;
                Kd[i] = 1.5;
                Kp[i] = 100; //5.0;
                Kd[i] = 2;
                qDes[i] = jointLinearInterpolation(init_joint_pos[i], jumpObj.default_joint_pos[i], rate);
                dqDes[i] = 0;
                tauDes[i] = 0;
            }
        }

        // double check objective reset in case we had to move the robot 
        if( motiontime >= time_vector[0]*1000 - 10 && motiontime < time_vector[0]*1000){
            jumpObj.resetObjectiveParams();
        }

        // third, start the jumping tests
        if( motiontime >= time_vector[0]*1000){
            // all control logic
            jumpObj.getControl();

            // set corresponding commands 
            for (int i=0; i<12; i++){
                qDes[i] = 0;
                dqDes[i] = 0;
                Kp[i] = 0.;
                Kd[i] = lowCmd.motorCmd[i].Kd;
                tauDes[i] = lowCmd.motorCmd[i].tau;
            }   
        
        }

        if ( !(motiontime % 1000)){
            printf("Time: %i \n", motiontime);
        }
        // original without BO 
        // if (motiontime >= (time_vector[1])*1000){
        //     printf("Reset robot to initial config, and jump again \n");
        //     jumpObj.init();
        //     motiontime=10;
        //     rate_count=0;
        //     for (int i=0; i < 12; i++){
        //         init_joint_pos[i] = lowState.motorState[i].q;
        //     }
        // }

        // reset for BO... 
        if (motiontime >= (time_vector[1])*1000){
            printf("Reset robot to initial config, and jump again... \n");

            // do BO 
            jumpObj.getSendObjective();
            if (jumpObj.do_BO){
                std::cout << jumpObj.seResult.rpy.transpose() << std::endl; 
                //while() don't have new parameters, sleep
                bool new_params = false;
                rate_count = 0;
                while(true && rate_count < 100 ){ // && num_resets < 99
                    ros::param::get("new_params", new_params);
                    // std::cout << "new params yet...? " << new_params << std::endl;
                    if (new_params || !jumpObj.do_BO){
                        break;
                    }
                    jumpObj.runStateEstimation();
                    rate_count++;
                    usleep(10000);
                }

                ros::param::set("new_params", false);
            }

            jumpObj.runStateEstimation();
            if (jumpObj.do_BO){
                jumpObj.updateParams(); 
            }
            jumpObj.resetObjectiveParams();
            jumpObj.init();

            // jumpObj.init();
            motiontime=10;
            rate_count=0;
            for (int i=0; i < 12; i++){
                init_joint_pos[i] = lowState.motorState[i].q;
            }
            printf("... all done with reset \n");
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


    if(motiontime > 8){
        // EXTREMELY IMPORTANT! NO CHANGES ALLOWED WITHOUT EXPLICIT PERMISSION FROM GUILLAUME OR MILAD
        safe.PositionLimit(cmd);
        int res1 = safe.PowerProtect(cmd, state, 7); // % of power allowed to use [above 5 NEVER ALLOWED without explicit permission]
        // You can uncomment it for position protection
        // int res2 = safe.PositionProtect(cmd, state, 0.087);
        if(res1 < 0) exit(-1);
    }

    udp.SetSend(cmd);
    ros::spinOnce();

}


int main(int argc, char **argv)
{
    // std::cout << "Communication level is set to LOW-level." << std::endl
    //           << "WARNING: Make sure the robot is hung up." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
    
    ros::init(argc, argv, "go1_jump2_lcm");

    Custom custom(LOWLEVEL);
    custom.init();

    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, boost::bind(&Custom::UDPRecv,      &custom));

    LoopFunc loop_writeOut("write_data",   0.001, boost::bind(&Custom::writeOutState,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    loop_writeOut.start();

    // while(1){
    //     sleep(10);
    // };
    // this fixes issue with CTRL+C not working
    while(ros::ok()){
        ros::spinOnce();
    };

    return 0; 
}
