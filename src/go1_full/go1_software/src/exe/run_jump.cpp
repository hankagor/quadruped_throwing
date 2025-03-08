/************************************************************************

Test jumping. The "setup" is standard and can mostly be copied between
different files. You MUST try any control algorithms with the simulated 
sensors in Gazebo BEFORE trying on the hardware. 

Implements simple half sine waves for jumping forward, laterally, twist,
either for a single jump, or continuous jumping. 
Follows from Legged Robots course single leg jumping. 

************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <fstream>

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>
#include "body.h" 
#include "../include/CurrentState.h"
#include "../../include/OrientationEstimator.h"
#include "../../include/PositionVelocityEstimator.h"
#include "../../include/ContactEstimator.h"
#include "../../include/LegController.h"
#include "../../include/ControlFSMData.h"
#include "../../CPG/HopfPolar.h"
#include "../../include/Math/MathUtilities.h"

using namespace std;
using namespace unitree_model;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go1_test_jump");

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;

    CurrentState listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(1); // one thread
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    ros::ServiceClient set_model_state_serv;
    ros::Rate loop_rate(1000);
    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

    // add simulation reset, so we don't have to keep relaunching ROS if robot falls over
    set_model_state_serv = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    motion_reset();
    usleep(1000000);
    gazebo_msgs::SetModelState model_state;
    model_state.request.model_state.model_name =  robot_name + "_gazebo" ;//"laikago_gazebo";
    model_state.request.model_state.pose.position.z = 0.5;
    //model_state.request.model_state.pose.position.x = -5;
    std::cout << "set model" << std::endl;
    std::cout << set_model_state_serv.call(model_state) << std::endl;
    usleep(1000000);

    // select Quadruped
    std::cout << "set quadruped" << std::endl;
    Quadruped quad ;
    quad.setQuadruped(1); // 1 for Go1, 2 for A1

    // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    std::cout << "start Leg Controller" << std::endl;
    LegController* legController = new LegController(quad);
    legController->zeroCommand();
    sendServoCmd();

    // start state estimation
    std::cout << "start state estimate" << std::endl;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    
    // change this to test algorithms using simulated sensors BEFORE trying on hardware
    if (0){
        // cheat data (ground truth)
        stateEstimator->addEstimator<ContactEstimator>();
        stateEstimator->addEstimator<CheaterOrientationEstimator>();
        stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    }
    else{
        // using sensors 
        stateEstimator->addEstimator<ContactEstimator>();
        stateEstimator->addEstimator<VectorNavOrientationEstimator>();
        stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();
    }

    // initialize FSMData
    std::cout << "start controlFSMData" << std::endl;
    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;

    motion_init();

    // start CPG
    HopfPolar cpg;
    cpg.init();
    std::cout << "CPG started" << std::endl;

    legController->updateData();
    stateEstimator->run();

    //cpg related vectors
    Vec4<double> x_out, z_out;
    double sideSign[4] = {-1, 1, -1, 1};
    double foot_y = 0.08;
    double foot_y2 = 0.1;

    Mat3<double> kpJoint = Mat3<double>::Zero(); 
    Mat3<double> kdJoint = Mat3<double>::Zero(); 
    kpJoint.diagonal() << 100, 100, 100;
    kdJoint.diagonal() << 2, 2, 2;

    bool ADD_CARTESIAN_PD = true;
    Mat3<double> kpCartesian = Mat3<double>::Zero(); 
    Mat3<double> kdCartesian = Mat3<double>::Zero(); 
    kpCartesian.diagonal() << 1000, 1000, 600;
    kdCartesian.diagonal() << 10, 10, 10;

    kpCartesian.diagonal() << 400, 400, 400;
    kdCartesian.diagonal() << 8, 8, 8;
    
    motion_reset();
    
    std::cout << "move to jump start position" << std::endl;
    double thigh = M_PI/2.7;
    double calf = -2*thigh;
    double pos[12] = {0.0, thigh, calf, -0.0, thigh, calf, 
                      0.0, thigh, calf, -0.0, thigh, calf};
    moveAllPosition(pos, 5000);
    sendServoCmd();
    std::cout << "... in init position" << std::endl;
    usleep(3000000);

    param_reset();

    // write out
    // int full_state_len = 76; // however many states you would like 
    // std::ofstream full_traj("full_state_gazebo_cpg.txt");
    // std::vector<double> full_state(full_state_len);


    // define jumping trajectory 
    bool SINGLE_JUMP = true;
    bool x_jump = false;
    bool y_jump = false;
    bool twist_jump = false;

    // jump time and vector 
    int NUM_SECONDS = 5;
    int traj_len = NUM_SECONDS*1000 + 1;
    std::vector<double> t(traj_len);
    for (int i=0; i<traj_len; i++){
        t[i] = 0.001*i;
    }

    // optimize these 
    double Fx_max = 60;  //     max peak force in X direction
    double Fy_max = 60;  //     max peak force in Y direction
    double Fz_max = 150; //     max peak force in Z direction
    double f = 0.85;      //     frequency
    double nom_z = 0.16;

    // XYZ force trajectories
    std::vector<double> force_traj_x(traj_len);
    std::vector<double> force_traj_y(traj_len);
    std::vector<double> force_traj_z(traj_len);

    // design force profile (see Legged Robots course slides)
    for (int i=0; i<traj_len; i++){
        force_traj_x[i] = Fx_max * std::sin(2*M_PI * f * t[i]);
        force_traj_y[i] = Fy_max * std::sin(2*M_PI * f * t[i]);
        force_traj_z[i] = Fz_max * std::sin(2*M_PI * f * t[i]);
        // don't apply force in the air
        if (force_traj_z[i] > 0){
            force_traj_x[i] = 0;
            force_traj_y[i] = 0;
            force_traj_z[i] = 0 ;
        }
        // zero out rest of trajectory for a single jump
        if (SINGLE_JUMP && i > 1/f/0.001){
            force_traj_x[i] = 0;
            force_traj_y[i] = 0;
            force_traj_z[i] = 0;
        }
    }

    // zero out X or Y if jumping in place 
    if (!x_jump) for (int i=0; i<traj_len; i++) force_traj_x[i] = 0;
    if (!y_jump) for (int i=0; i<traj_len; i++) force_traj_y[i] = 0;


    int counter = 0;
    while (ros::ok() && counter<10000 ){
        // update current robot state
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        auto& seResult = _controlData->_stateEstimator->getResult();

        /*
        control logic
        */
        Vec4<double> forceMultipliers;  // add forces to counteract pitch/roll 
        forceMultipliers << 1, 1, 1, 1; // TODO: whole-body controller instead
        double roll_factor = 5;
        double pitch_factor = 5;
        //counteract roll
        if (seResult.rpy[0] < 0 ){
            forceMultipliers[1] += -roll_factor * seResult.rpy[0];
            forceMultipliers[3] += -roll_factor * seResult.rpy[0];
        }
        else{
            forceMultipliers[0] += roll_factor * seResult.rpy[0];
            forceMultipliers[2] += roll_factor * seResult.rpy[0];
        }
        //counteract pitch
        if (seResult.rpy[1] < 0 ){
            forceMultipliers[2] += -pitch_factor * seResult.rpy[1];
            forceMultipliers[3] += -pitch_factor * seResult.rpy[1];
        }
        else{
            forceMultipliers[0] += pitch_factor * seResult.rpy[1];
            forceMultipliers[1] += pitch_factor * seResult.rpy[1];
        }
        std::cout << seResult.rpy[0] << " " << seResult.rpy[1] << " " << seResult.rpy[2] << "\n";

        // loop through legs 
        for (int i=0; i<4; i++){
            // desired foot position and corresponding joint angles
            Vec3<double> pDes, qDes, tau;
            pDes << -0.02, sideSign[i] * foot_y2, -nom_z; // arbitrary (optimize?)
            
            // Cartesian PD on nominal foot position
            for (int j = 0; j < 3; j++){
                _controlData->_legController->commands[i].pDes[j] = pDes[j];
                _controlData->_legController->commands[i].vDes[j] = 0;
                _controlData->_legController->commands[i].kpCartesian(j,j) = kpCartesian(j,j);
                _controlData->_legController->commands[i].kdCartesian(j,j) = kdCartesian(j,j);
                _controlData->_legController->commands[i].feedforwardForce[j]  = 0;
                // add some joint damping for hardware
                lowCmd.motorCmd[i*3+j].dq = 0;
                lowCmd.motorCmd[i*3+j].Kd = 0.5;
            }

            // add force contributions during jumping trajectory 
            if (counter < traj_len){
                _controlData->_legController->commands[i].feedforwardForce[0] = force_traj_x[counter];
                if (twist_jump){
                    if (i<2)
                        _controlData->_legController->commands[i].feedforwardForce[1] = -force_traj_y[counter];
                    else
                        _controlData->_legController->commands[i].feedforwardForce[1] = force_traj_y[counter];
                }
                else{
                        _controlData->_legController->commands[i].feedforwardForce[1] = force_traj_y[counter];
                }
                _controlData->_legController->commands[i].feedforwardForce[2] = forceMultipliers[i] * force_traj_z[counter];
            }
            
            _controlData->_legController->updateCommandNoSend();
        }
        
        // [TODO] write a function to save data, by querying the below for body states ...
        // auto& seResult = _controlData->_stateEstimator->getResult();
        // ... and querying joints with lowState.motorState[jnt_idx].q... etc.
        // can also include CPG states for debugging 

        // send command to motors
        sendServoCmd();

        loop_rate.sleep();
        counter++;
    }

    std::cout << "finished" << std::endl;
    //delete quad;
    //delete &pyb_interface_obj;
    delete legController;
    delete stateEstimator;
    delete _controlData;

    return 0;
}
