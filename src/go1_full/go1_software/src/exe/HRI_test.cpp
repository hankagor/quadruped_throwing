

/************************************************************************

rear-up test

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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
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
#include "../../jumping/HRI.h"
#include "../../include/Math/MathUtilities.h"

using namespace std;
using namespace unitree_model;


bool thrown = false;

void model_state_callback(const gazebo_msgs::ModelStates::ConstPtr& data) {
    try {
        auto it = std::find(data->name.begin(), data->name.end(), "smaller_box");
        if (it != data->name.end()) {
            int index = std::distance(data->name.begin(), it);
            geometry_msgs::Pose position = data->pose[index];
            if (position.position.x < 0.2) {
                thrown = true;
            }
            if (position.position.z < 0.15 && position.position.x > 0.2 && thrown) {
                thrown = false;
                ROS_INFO("smaller_box Position: x=%.2f, y=%.2f, z=%.2f", position.position.x, position.position.y, position.position.z);
                
            }
        }
    } catch (...) {
        ROS_WARN("Model 'smaller_box' not found in model states");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go1_test_cpg");

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;

    CurrentState listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(1); // one thread
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    ros::Publisher tau_pub[6], pos_pub[4], vel_pub[4];
    ros::ServiceClient set_model_state_serv;
    ros::Rate loop_rate(1000);
    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    for (int i = 0; i < 6; i++){
        std::string tau = "/tau";
        char idx = ('0' + i%10);
        tau += idx;
        tau_pub[i] = n.advertise<std_msgs::Float64>(tau, 1);
    }
    pos_pub[0] = n.advertise<std_msgs::Float64>("pdesx", 1);
    pos_pub[1] = n.advertise<std_msgs::Float64>("posx", 1);
    pos_pub[2] = n.advertise<std_msgs::Float64>("pdesz", 1);
    pos_pub[3] = n.advertise<std_msgs::Float64>("posz", 1);

    vel_pub[0] = n.advertise<std_msgs::Float64>("vdesx", 1);
    vel_pub[1] = n.advertise<std_msgs::Float64>("velx", 1);
    vel_pub[2] = n.advertise<std_msgs::Float64>("vdesz", 1);
    vel_pub[3] = n.advertise<std_msgs::Float64>("velz", 1);
    
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
    ros::Publisher p_pub[2];
        ros::Publisher v_pub[2];
        p_pub[0] = n.advertise<std_msgs::Float64>("/pdes", 1);
        p_pub[1] = n.advertise<std_msgs::Float64>("/pos", 1);
        v_pub[0] = n.advertise<std_msgs::Float64>("/vdes", 1);
        v_pub[1] = n.advertise<std_msgs::Float64>("/vel", 1);
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
    ros::Subscriber sub = n.subscribe("/gazebo/model_states", 10, model_state_callback);
    
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
    legController->updateData();
    stateEstimator->run();
    motion_reset();

    HRI hri;
    hri._controlData = _controlData;
    hri.init();
    
    // load trajectory 
    // hri.readTraj("src/go1_full/go1_software/jumping/rear_up_hw1.csv");
    hri.readTraj("src/go1_full/go1_software/jumping/rear_up_sim.csv");
    // hri.readTraj("src/go1_full/go1_software/jumping/front_up.csv");
    std::cout << "move to start position" << std::endl;

    // moveAllPosition(hri.default_joint_pos, 2000);
    // sendServoCmd();
    // std::cout << "... in init position" << std::endl;
    // usleep(3000000);
    // param_reset();
    

    // write out
    // int full_state_len = 76; // however many states you would like 
    // std::ofstream full_traj("full_state_gazebo_cpg.txt");
    // std::vector<double> full_state(full_state_len);

    moveAllPosition(hri.three_leg_start, 2000);
    sendServoCmd();
    usleep(3000000);
    // param_reset();

    /* // 3 keg handshake
    if (ros::ok() ){
        std::cout << "Test 3 leg" << std::endl;
        for(int i = 0; i < 10000; i++)
        {
            // update current robot state
            hri.runStateEstimation();

            // move 3 legs 
            hri.setLegControllerCommandsMove3Legs();
            
            // send command to motors
            sendServoCmd();

            loop_rate.sleep();
        } 
    }
    */ 

    // rear up and catch 
    std::cout << "Reset position" << std::endl;
    for (int i =0; i < 12; i++)
    {
        lowCmd.motorCmd[i].Kp = hri.kpJoint(0,0);
        lowCmd.motorCmd[i].Kd = hri.kdJoint(0,0);
    }
    moveAllPosition(hri.default_joint_pos, 2000);
    sendServoCmd();
    usleep(3000000);

    if (ros::ok() ){
        std::cout << "Start rear up" << std::endl;
        for(int i = 0; i < hri.full_opt_N; i++)
        {
            // update current robot state
            hri.runStateEstimation();

            // get trajectory action
            hri.setLegControllerCommandsFromTrajIndex(i);

            // write out full state 
            // hri.getFullState(full_state);
            // for(int j=0; j<full_state_len; j++)
            //     full_traj << full_state[j] << " ";
            // full_traj << "\n";
            
            // send command to motors
            sendServoCmd();

            loop_rate.sleep();
        } 

        std::cout << "Robot is standing..." << std::endl;
        // wait the equivalent of 5 stand ups before moving back down
        int i =0;
        while(i < 2000){
            loop_rate.sleep(); // wait a second to settle
            i++;
        }
        std::cout << "... done settling." << std::endl;
        i=0;
        int closed = 0;
        while(ros::ok()) //i < 10*hri.full_opt_N || abs(lowState.motorState[0].q) > 0.02 || abs(lowState.motorState[3].q) > 0.02)
        {

            if (hri.throwing && hri.max_vel/2 <= _controlData->_legController->data[0].v(0) && hri.close_flag > 0){
                hri.close_flag = 0;
                ROS_INFO("Box thrown!");
            }
            if (!closed && hri.close_flag == 2)
                closed = 1;
            if (hri.throwing && closed && !hri.close_flag){
                closed = 0;
                // std_msgs::Float64 opening;
                // opening.data = 4;
                // vel_pub[0].publish(opening);
                // opening.data = 0.0;
                // pos_pub[0].publish(opening);
            }
            hri.runStateEstimation();
            hri.setLegControllerCommandsFromOscillators();
            
            sendServoCmd();
            // hri._controlData->_legController->publishPosCmd();
            // for (int k = 0; k < 6; k++){
            //     std_msgs::Float64 tau_msg;
            //     tau_msg.data = lowCmd.motorCmd[k].tau;

            //     tau_pub[k].publish(tau_msg);
            // }
            loop_rate.sleep();
            i++;
            if (!hri.throwing)
                continue;
            int k = 0;
            // std_msgs::Float64 pdes, pos, vdes, vel;
            // pdes.data = _controlData->_legController->commands[0].pDes(k);
            // pos.data = _controlData->_legController->data[0].p(k);
            // vdes.data = _controlData->_legController->commands[0].vDes(k)* sqrt(2)/2;
            // vel.data = _controlData->_legController->data[0].v(k);
            // pos_pub[0].publish(pdes);
            // pos_pub[1].publish(pos);
            // vel_pub[0].publish(vdes);
            // vel_pub[1].publish(vel);
            // k = 2;
            // pdes.data = _controlData->_legController->commands[0].pDes(k);
            // pos.data = _controlData->_legController->data[0].p(k);
            // vdes.data = _controlData->_legController->commands[0].vDes(k) * sqrt(2)/2;
            // vel.data = _controlData->_legController->data[0].v(k);
            // pos_pub[2].publish(pdes);
            // pos_pub[3].publish(pos);
            // vel_pub[2].publish(vdes);
            // vel_pub[3].publish(vel);
            // std::cout << _controlData->_legController->data[0].p[1] -  _controlData->_legController->data[1].p[1] << std::endl;
            // if (i == 1000){
            //     hri.spawnObject();
            //     ROS_INFO("left leg %.2f x %.2f x %.2f", hri._controlData->_legController->data[1].p[0], hri._controlData->_legController->data[1].p[1], hri._controlData->_legController->data[1].p[2]);
            //     ROS_INFO("right leg %.2f x %.2f x %.2f", hri._controlData->_legController->data[0].p[0], hri._controlData->_legController->data[0].p[1], hri._controlData->_legController->data[0].p[2]);
    
            // }
            
        }
        hri.moveToTrajInitPos();

    }
    

    std::cout << "finished" << std::endl;
    delete legController;
    delete stateEstimator;
    delete _controlData;

    return 0;
}
