/*****************************************************************
 Test CPG-RL gaits with LCM on hardware

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
// #include "../../CPG/HopfPolar.h"
#include "../../include/Math/MathUtilities.h"
// #include "../../CPG/JumpingObj.h"
#include "../../DRL/IsaacGymTorchInterface.h"

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
                            cpg_state(cpg_state_len),
                            full_traj("/data/bellegar/quadruped_logs/full_state_hw_rl.txt"),
                            obs_traj("/data/bellegar/quadruped_logs/real_obs.txt"),
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
    float dt = 0.001;     // 0.001~0.01, originally 0.002
    // std::chrono::steady_clock::time_point start, end;
    // int time_vector[5] = { 10, 10, 11, 14, 17 }; // change gaits test 
    int time_vector[5] = { 5, 8, 11}; //11, 14, 17 }; // change gaits test 
    int last_time = 14;
    bool end_early_flag = false;
    bool do_gait_transitions = true;

    double v[3] = {0.4, 0., 0.};

    float footForceZero[4] = {1000.0, 1000.0, 1000.0, 1000.0}; // fix "zero" value, sensors read 100+ N even when in air
    // float footForcePrev[4] = {0.0}; // fix "zero" value, sensors read 100+ N even when in air
    double init_joint_pos[12] = {0};
    // double default_joint_pos[12] = { 0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3  };

    double default_joint_pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                                    0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    double default_floor_pos[12] = {-0.0, 1.6, -2.7, -0.0, 1.6, -2.7, 
                                     0.0, 1.6, -2.7, -0.0, 1.6, -2.7};

    // write out
    int full_state_len = 76;
    int cpg_state_len = 24;
    std::ofstream full_traj; //("full_state.txt");
    std::vector<double> full_state; //(full_state_len);
    std::vector<double> cpg_state; //(cpg_state_len);
    std::ofstream obs_traj;
    int obs_len;

    // all quadruped objects
    Quadruped quad;
    StateEstimate stateEstimate;
    LegController* legController;
    StateEstimatorContainer* stateEstimator;
    ControlFSMData* _controlData;
    CurrentState listen_publish_obj;
    IsaacGymTorchInterface pyb_interface_obj;

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
    if (1){
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


    // init cpg-rl object
    pyb_interface_obj._controlData = _controlData;
    obs_len = pyb_interface_obj.obs_len;
    pyb_interface_obj.hardwareFlag = true;
    std::cout << "IsaacGymTorchInterface started" << std::endl;

    // start = std::chrono::steady_clock::now();
    
}

void Custom::writeOutState(){
    pyb_interface_obj.getFullState(full_state);
    pyb_interface_obj.getCPGState(cpg_state);
    for(int j=0; j<full_state_len; j++)
        full_traj << full_state[j] << " ";
    for(int j=0; j<cpg_state_len; j++)
        full_traj << cpg_state[j] << " ";
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
        lowState.footForce[i] = (lowState.footForce[i] - footForceZero[i]) / 5 ; /// 2.0; // / 9.8; // is this more reasonable? 
        // lowState.footForce[i] = lowState.footForce[i] / 10; //9.8;
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
    _controlData->_legController->updateData();
    _controlData->_stateEstimator->run();
    auto& seResult = _controlData->_stateEstimator->getResult();

    // printf("%d  %f\n", motiontime, lowState.motorState[FR_2].q);
    // printf("lowState   %d  %f\n", motiontime, lowState.imu.quaternion[2]); 
    // printf("RecvLowROS %d  %f\n", motiontime, RecvLowROS.imu.quaternion[2]);

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
                qDes[i] = jointLinearInterpolation(init_joint_pos[i], default_joint_pos[i], rate);
                dqDes[i] = 0;
                tauDes[i] = 0;

                // this is good for trot and pace 
                Kp[i] = 55;  
                Kd[i] = 1; 

                // Kp[i] = 65;  
                // Kd[i] = 1.2; 
            }
        }

        // hold for a few seconds to have a good initial position
        if( motiontime >= 3000 && motiontime < time_vector[0]*1000){
            for (int i=0; i<12; i++){
                qDes[i] = default_joint_pos[i];
            }
        }

        // third, start the locomotion tests
        if( motiontime >= time_vector[0]*1000){
            // all control logic

            /*
            Gaits:
            1 bounding
            2 trotting
            3 walking
            4 pacing
            5 pronking
            6 gallop1
            7 gallop2 
            ["TROT", "WALK", "PACE", "BOUND", "PRONK", "GALLOP"]
            */
            if (do_gait_transitions){
                if( motiontime >= time_vector[1]*1000){
                    v[0] = 0.6;
                    pyb_interface_obj.cpg.SetGait(2); // change pace 
                }
                if( motiontime >= time_vector[2]*1000){
                    v[0] = 0.8;
                    pyb_interface_obj.cpg.SetGait(4); // change bound (1), gallop (6 or 7)
                }
                // test later (pace and trot first are fine )
                // if( motiontime >= time_vector[2]*1000){
                //     v[0] = 1.2;
                //     pyb_interface_obj.cpg.SetGait(5); // change pronk 
                // }
                // if( motiontime >= time_vector[3]*1000){
                //     v[0] = 1.5;
                //     pyb_interface_obj.cpg.SetGait(1); // change bound (1), gallop (6 or 7)
                // }
            }
            
            v[2] = clip(-seResult.rpy[2], -0.2, 0.2); // steering to go straight 
            pyb_interface_obj.setCommand(v);
            
            if (motiontime % 10 == 0){ // choose new high level command every X*dt s 
        	    pyb_interface_obj.computeActionCPG();

                for (int j=0; j < obs_len; j++)
                    obs_traj << pyb_interface_obj.rl_obs_data[0][j].item<float>() << " ";
                for (int j=0; j < 12; j++)
                    obs_traj << lowState.motorState[j].tauEst << " ";
                obs_traj << "\n";
            }
            pyb_interface_obj.generalSetLegCommandsRLCPG();

            // set corresponding commands 
            for (int i=0; i<12; i++){
                qDes[i] = lowCmd.motorCmd[i].q;
                // dqDes[i] = 0;
                // Kp[i] = 0.;
                // Kd[i] = lowCmd.motorCmd[i].Kd;
                // tauDes[i] = lowCmd.motorCmd[i].tau;
            }   
        
        }

        if ( !(motiontime % 1000)){
            printf("Time: %i \n", motiontime);
        }


        // stop after x seconds 
        if (motiontime >= last_time*1000){
            for (int i=0; i < 12; i++){
                qDes[i] = lowState.motorState[i].q;
                dqDes[i] = 0;
                tauDes[i] = 0;
                Kp[i] = 25.;
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
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    ros::init(argc, argv, "go1_cpg_rl_lcm");

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
