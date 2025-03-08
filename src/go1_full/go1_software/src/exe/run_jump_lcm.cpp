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
                            force_traj_x(traj_len),
                            force_traj_y(traj_len),
                            force_traj_z(traj_len),
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
    int traj_counter = 0; // index through trajectory more easily 
    float dt = 0.001;     // 0.001~0.01, originally 0.002
    std::chrono::steady_clock::time_point start, end;
    int time_vector[2] = { 5, 12 }; // times to switch modes in seconds
    int last_time = 12;

    float footForceZero[4] = {1000.0, 1000.0, 1000.0, 1000.0}; // fix "zero" value, sensors read 100+ N even when in air
    // float footForcePrev[4] = {0.0}; // fix "zero" value, sensors read 100+ N even when in air
    double init_joint_pos[12] = {0};
    // double default_joint_pos[12] = { 0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3,
    //                                  0., M_PI/3, -2*M_PI/3  };
    double thigh = M_PI/2.7;
    double calf = -2*thigh;
    double default_joint_pos[12] = {0.0, thigh, calf, -0.0, thigh, calf, 
                                    0.0, thigh, calf, -0.0, thigh, calf};

    double default_floor_pos[12] = {-0.0, 1.6, -2.7, -0.0, 1.6, -2.7, 
                                     0.0, 1.6, -2.7, -0.0, 1.6, -2.7};

    double current_dof_pos[12] = {0};
    double current_dof_vel[12] = {0};
    int hips[4] = { 0, 3, 6, 9 };

    // jumping test
    // what kind of jump (TODO: improve logic here)
    bool SINGLE_JUMP = true;
    bool x_jump = false;
    bool y_jump = false;
    bool twist_jump = false;
    // time vector 
    int NUM_SECONDS = time_vector[1] - time_vector[0];
    int traj_len = NUM_SECONDS*1000 + 1;
    std::vector<double> t; 
    // force trajectoriess
    std::vector<double> force_traj_x;
    std::vector<double> force_traj_y;
    std::vector<double> force_traj_z;
    
    bool first_time = true;
    double Fx_max = 60;  //     max peak force in X direction
    double Fy_max = 60;  //     max peak force in Y direction
    double Fz_max = 190;  //     max peak force in Z direction 
    double f = 0.85;      //     frequency
    double nom_z = 0.2; //     default height
    double nom_z2 = 0.25; // for landing

    // working parametersL Fz_max = 140, f=0.85, z=0.2, nom_z2=0.25, 
    //kpCartesian.diagonal() << 400, 400, 500;
    //        kdCartesian.diagonal() << 8, 8, 10;
    // 160 also worked

    // CPG 
    HopfPolar cpg;
    Vec4<double> x_out, z_out;
    double sideSign[4] = {-1, 1, -1, 1};
    double foot_y = 0.08;
    double foot_y2 = 0.1;

    double hiplen = 0.08; //0.08; //0.04675; // 0.08 +
    double hiplen2 = 0.04675;
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


    // design force profile (see Legged Robots course slides)
    for (int i=0; i<traj_len; i++){
        t[i] = dt*i;
        force_traj_x[i] = Fx_max * std::sin(2*M_PI * f * t[i]);
        force_traj_y[i] = Fy_max * std::sin(2*M_PI * f * t[i]);
        force_traj_z[i] = Fz_max * std::sin(2*M_PI * f * t[i]);
        // std::cout << " " << Fz_max * std::sin(2*M_PI * f * t[i]);
        // std::cout << " " << std::sin(2*M_PI * f * t[i]);
        // don't apply force in the air
        if (force_traj_z[i] > 0){
            force_traj_x[i] = 0;
            force_traj_y[i] = 0;
            force_traj_z[i] = 0;//-20; // normal force (approx)
        }
        else{
            force_traj_z[i] -= 20;
        }
        // zero our rest of trajectory for a single jump
        if (SINGLE_JUMP && i > 1/f/0.001){
            force_traj_x[i] = 0;
            force_traj_y[i] = 0;
            force_traj_z[i] = 0;
        }
        // std::cout << "after " << force_traj_z[i] << std::endl;
    }
    // zero out X or Y if jumping in place 
    if (!x_jump) for (int i=0; i<traj_len; i++) force_traj_x[i] = 0;
    if (!y_jump) for (int i=0; i<traj_len; i++) force_traj_y[i] = 0;

    std::cout << "Force trajectories initialized" << std::endl;

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
    // std::cout << "footForce PRE-CLIP  "    << lowState.footForce[0] << " "
    //                              << lowState.footForce[1] << " "
    //                              << lowState.footForce[2] << " "
    //                              << lowState.footForce[3] << " " << std::endl;
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
    std::cout << "footForce POST-CLIP "    << lowState.footForce[0] << " "
                                 << lowState.footForce[1] << " "
                                 << lowState.footForce[2] << " "
                                 << lowState.footForce[3] << " " << std::endl;
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
        if( motiontime >= time_vector[0]*1000){ //time_vector[0]*1000){

            Mat3<double> kpCartesian = Mat3<double>::Zero(); 
            Mat3<double> kdCartesian = Mat3<double>::Zero(); 
            kpCartesian.diagonal() << 400, 400, 500;
            kdCartesian.diagonal() << 8, 8, 10;

            Vec4<double> forceMultipliers; // add forces to counteract pitch/roll 
            forceMultipliers << 1, 1, 1, 1;
            double roll_factor = 5;
            double pitch_factor = 5;
            if (seResult.rpy[0] < 0 ){
                //counteract roll
                forceMultipliers[1] += -roll_factor * seResult.rpy[0];
                forceMultipliers[3] += -roll_factor * seResult.rpy[0];

                // forceMultipliers[0] +=  roll_factor * seResult.rpy[0];
                // forceMultipliers[2] += roll_factor * seResult.rpy[0];

            }
            else{
                forceMultipliers[0] += roll_factor * seResult.rpy[0];
                forceMultipliers[2] += roll_factor * seResult.rpy[0];
            }
            if (seResult.rpy[1] < 0 ){
                //counteract pitch
                forceMultipliers[2] += -pitch_factor * seResult.rpy[1];
                forceMultipliers[3] += -pitch_factor * seResult.rpy[1];

                // forceMultipliers[0] +=  pitch_factor * seResult.rpy[1];
                // forceMultipliers[1] +=  pitch_factor * seResult.rpy[1];
            }
            else{
                forceMultipliers[0] += pitch_factor * seResult.rpy[1];
                forceMultipliers[1] += pitch_factor * seResult.rpy[1];
            }
            
            std::cout << seResult.rpy[0] << " " << seResult.rpy[1] << " " << seResult.rpy[2] << "\n";

            for (int i=0; i<4; i++){
                // desired foot position and corresponding joint angles
                Vec3<double> pDes, qDesLeg, tau, force_i;
                pDes << 0, sideSign[i] * foot_y2, -nom_z; // arbitrary (optimize?)

                force_i << 0, 0, 0;
                if (traj_counter < traj_len )
                    force_i[2] = forceMultipliers[i] * force_traj_z[traj_counter];

                
                // TODO: add the force contribution, just testing right now 

                // Cartesian PD on nominal foot position
                for (int j = 0; j < 3; j++){
                    _controlData->_legController->commands[i].pDes[j] = pDes[j];
                    _controlData->_legController->commands[i].vDes[j] = 0;
                    _controlData->_legController->commands[i].kpCartesian(j,j) = kpCartesian(j,j);
                    _controlData->_legController->commands[i].kdCartesian(j,j) = kdCartesian(j,j);
                    _controlData->_legController->commands[i].feedforwardForce[j]  = force_i[j];
                    // zero joint PG, and/or add some joint damping for hardware
                    qDes[i*3+j] = 0;
                    dqDes[i*3+j] = 0;
                    Kp[i*3+j] = 0.;
                    Kd[i*3+j] = 0.5;
                    if (lowState.footForce[i] ==0){
                        Kd[i*3+j] = 1.5;
                        nom_z = nom_z2; // for next time, lower desired foot location to brace better
                    }
                    
                }

            }
            traj_counter++;


            // actually compute torque contribution 
            _controlData->_legController->updateCommandNoSend(); 
            for (int j=0; j<12; j++){
                // tauDes[j] = clip(lowCmd.motorCmd[j].tau, -5.0f, 5.0f);
                tauDes[j] = clip(lowCmd.motorCmd[j].tau, -20.0f, 20.0f);
                printf("%f ",tauDes[j]);
            }
            printf("\n");
            
        
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
        int res1 = safe.PowerProtect(cmd, state, 6); // % of power allowed to use [above 5 NEVER ALLOWED without explicit permission]
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
