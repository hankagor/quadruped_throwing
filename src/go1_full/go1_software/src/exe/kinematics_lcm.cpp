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
                            base_motion(traj_len),
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
    int time_vector[2] = { 3, 20 }; // times to switch modes in seconds
    int last_time = 20;

    float footForceZero[4] = {1000.0, 1000.0, 1000.0, 1000.0}; // fix "zero" value, sensors read 100+ N even when in air
    // float footForcePrev[4] = {0.0}; // fix "zero" value, sensors read 100+ N even when in air
    double init_joint_pos[12] = {0};
    double default_joint_pos[12] = { 0., M_PI/3, -2*M_PI/3,
                                     0., M_PI/3, -2*M_PI/3,
                                     0., M_PI/3, -2*M_PI/3,
                                     0., M_PI/3, -2*M_PI/3  };

    double default_floor_pos[12] = {-0.0, 1.6, -2.7, -0.0, 1.6, -2.7, 
                                     0.0, 1.6, -2.7, -0.0, 1.6, -2.7};

    double current_dof_pos[12] = {0};
    double current_dof_vel[12] = {0};
    int hips[4] = { 0, 3, 6, 9 };

    // Ekeberg
    int NUM_SECONDS = time_vector[1] - time_vector[0];
    int traj_len = NUM_SECONDS*1000 + 1;
    std::vector<double> t; 
    // rpy trajectories
    std::vector<double> base_motion;
    
    double f = 0.3;        // frequency 
    double max_rpy = 0.6;  // max roll, pitch, or yaw 
    int rpy = 0; // flag for in roll, pitch, or yaw 
    bool first_time = true;


    // CPG 
    HopfPolar cpg;
    Vec4<double> x_out, z_out;
    double sideSign[4] = {-1, 1, -1, 1};
    double foot_y = 0.08;

    double hiplen = 0.08; //0.08; //0.04675; // 0.08 +
    double hiplen2 = 0.04675;
    double nom_z = 0.22;
    double leg_offset_x = 0.1881;

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


    // rpy trajectory 
    for (int i=0; i<traj_len; i++){
        t[i] = dt*i;
        base_motion[i] = max_rpy * std::sin(2*M_PI * f * t[i]); 
        // std::cout << " " << std::sin(2*M_PI * f * t[i]);
    }
    std::cout << "Base motion init" << std::endl;

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
        if( motiontime >= 10 && motiontime < 3000 ) { //time_vector[0]*1000){
            rate_count++;
            double rate = rate_count/2990.0;  // needs count to 200
            for (int i=0; i<12; i++){
                Kp[i] = 55; //5.0;
                Kd[i] = 1.5;
                qDes[i] = jointLinearInterpolation(init_joint_pos[i], default_joint_pos[i], rate);
            }
        }

        // third, start the kinematic tests
        if( motiontime >= 3000){ //time_vector[0]*1000){

            for (int i=0; i<4; i++){
                // desired foot position and corresponding joint angles
                Vec3<double> pDes, qDesLeg;
                pDes << 0, sideSign[i] * foot_y , -nom_z; 
                double z = 0.0;

                
                if (motiontime < time_vector[0]*1000 + 5000){
                    if (first_time && std::abs(base_motion[motiontime % traj_len]) > 0.01)
                        break;
                    first_time = false;
                    rpy = 0;
                }
                else if(motiontime > time_vector[0]*1000 + 5000 && motiontime < time_vector[0]*1000 + 10000 ){
                    if (std::abs(base_motion[motiontime % traj_len]) < 0.01)
                        rpy = 1;
                }
                else{
                    if (std::abs(base_motion[motiontime % traj_len]) < 0.01)
                        rpy = 2;
                }

                // test roll
                if (rpy==0){
                    if (i==0 || i == 2){
                        z = nom_z - hiplen2*std::sin(base_motion[motiontime % traj_len]);
                        pDes << 0, 
                                sideSign[i] * foot_y * std::sin(base_motion[motiontime % traj_len]), 
                                -nom_z + hiplen2*std::sin(base_motion[motiontime % traj_len]);  
                    }
                    else{
                        z = nom_z + hiplen2*std::sin(base_motion[motiontime % traj_len]);
                        pDes << 0, 
                                sideSign[i] * foot_y * std::sin(base_motion[motiontime % traj_len]),
                                -nom_z - hiplen2*std::sin(base_motion[motiontime % traj_len]);   
                    }
                }

                // test pitch 
                if (rpy == 1){
                    if (i==0 || i == 1)
                        pDes << leg_offset_x*std::sin(base_motion[motiontime % traj_len]), 
                                sideSign[i] * foot_y , 
                                -nom_z + leg_offset_x*std::sin(base_motion[motiontime % traj_len]);
                    else
                        pDes << leg_offset_x*std::sin(base_motion[motiontime % traj_len]), 
                            sideSign[i] * foot_y , 
                            -nom_z - leg_offset_x*std::sin(base_motion[motiontime % traj_len]);
                }


                // test yaw 
                if (rpy==2 ){
                    if (i==0 || i == 1)
                        pDes << 0, sideSign[i] * foot_y - 0.1*std::sin(base_motion[motiontime % traj_len]), -nom_z;
                    else
                        pDes << 0, sideSign[i] * foot_y + 0.1*std::sin(base_motion[motiontime % traj_len]), -nom_z;
                    
                }
                
                computeInverseKinematics(_controlData->_legController->_quadruped, pDes, i, &qDesLeg);
                for (int j = 0; j < 3; j++){
                    // test joint PD control
                    qDes[i*3+j] = qDesLeg[j];
                }
                if(rpy==0) // for roll
                    qDes[i*3] = -base_motion[motiontime % traj_len];

                std::cout << "qDes " << i << " " << qDes[i*3] << " " << qDes[i*3+1] << " " << qDes[i*3+2]<< std::endl;
            }


            

            // std::cout << "\n";
            
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
    
    ros::init(argc, argv, "go1_kinematics_lcm");

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
