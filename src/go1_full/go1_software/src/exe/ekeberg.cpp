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
    if (1){
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

    Mat3<double> kpJoint = Mat3<double>::Zero(); 
    Mat3<double> kdJoint = Mat3<double>::Zero(); 
    kpJoint.diagonal() << 100, 100, 100;
    kdJoint.diagonal() << 2, 2, 2;

    bool ADD_CARTESIAN_PD = true;
    Mat3<double> kpCartesian = Mat3<double>::Zero(); 
    Mat3<double> kdCartesian = Mat3<double>::Zero(); 
    kpCartesian.diagonal() << 1000, 1000, 600;
    kdCartesian.diagonal() << 10, 10, 10;
    
    motion_reset();
    
    std::cout << "move to muscle init position" << std::endl;
    double thigh = M_PI/4;
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

    // default joint angles 
    double default_dof_pos[12] = {0.0, thigh, calf, -0.0, thigh, calf, 
                                  0.0, thigh, calf, -0.0, thigh, calf};
    double current_dof_pos[12] = {0};
    double current_dof_vel[12] = {0};
    for (int i =0; i<4; i++){
        for (int j = 0; j < 3; j++){
           current_dof_pos[3*i+j] =  _controlData->_legController->data[i].q[j];
           current_dof_vel[3*i+j] =  _controlData->_legController->data[i].qd[j];
        }
        
    }


    // jump time and vector 
    int NUM_SECONDS = 5;
    int traj_len = NUM_SECONDS*1000 + 1;
    std::vector<double> t(traj_len);
    for (int i=0; i<traj_len; i++){
        t[i] = 0.001*i;
    }

    // optimize these 
    double ME = 1;  //     max peak force in X direction
    double MF = 1;  //     max peak force in Y direction
    double f = 0.8;      //     frequency

    // XYZ force trajectories
    std::vector<double> me_traj(traj_len);
    std::vector<double> mf_traj(traj_len);

    // design force profile (see Legged Robots course slides)
    for (int i=0; i<traj_len; i++){
        me_traj[i] = 0.5 * (ME * std::sin(2*M_PI * f * t[i]) + 1);
        mf_traj[i] = 0.5 * (MF * std::sin(2*M_PI * f * t[i]) + 1);
    }



    int counter = 0;
    while (ros::ok() && counter<10000 ){
        // update current robot state
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();

        /*
        control logic
        */

        // test muscle model
        float alpha, beta, gamma, delta, ML, MR;
        alpha=2;//20;
        beta=5;//20;
        gamma=10;
        delta=1;

        // alpha=0;//20;
        // beta=1;//20;
        // gamma=1;
        // delta=1;
        alpha=0;//20;
        beta=5;//20;
        gamma=1;
        delta=2;

        alpha=10; beta=10; gamma=2; delta=2;

        alpha=15; beta=10; gamma=3; delta=1;

        alpha=15; beta=15; gamma=2; delta=1;
        
        ML = 0; MR = 0;
        ML = me_traj[counter]; MR = mf_traj[counter];
        auto& seResult = _controlData->_stateEstimator->getResult();
        for (int i =0; i<4; i++){
            for (int j = 0; j < 3; j++){
               current_dof_pos[3*i+j] =  _controlData->_legController->data[i].q[j];
               current_dof_vel[3*i+j] =  _controlData->_legController->data[i].qd[j];
            }     
        }

        double tau_limit = 10.0;
        for (int i=0; i<12; i++){
            lowCmd.motorCmd[i].q = 0; // won't affect due to gains at 0
            lowCmd.motorCmd[i].dq = 0;
            lowCmd.motorCmd[i].tau = clip(alpha * (ML - MR) 
                                        + beta * (ML + MR + gamma) * (default_dof_pos[i] - current_dof_pos[i] ) 
                                        + delta * (-current_dof_vel[i]), -15.0, 15.0);
            lowCmd.motorCmd[i].Kp = 0;
            lowCmd.motorCmd[i].Kd = 0;
        } 

        lowCmd.motorCmd[0].tau = 100*(default_dof_pos[0] - current_dof_pos[0] ) - 2* current_dof_vel[0];
        lowCmd.motorCmd[3].tau = 100*(default_dof_pos[3] - current_dof_pos[3] ) - 2* current_dof_vel[3];
        lowCmd.motorCmd[6].tau = 100*(default_dof_pos[6] - current_dof_pos[6] ) - 2* current_dof_vel[6];
        lowCmd.motorCmd[9].tau = 100*(default_dof_pos[9] - current_dof_pos[9] ) - 2* current_dof_vel[9];
        
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
