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
#include "../../CPG/JumpingObj.h"
#include "../../utils/CubeSpawner.h"
#include "../../include/Math/orientation_tools.h"
#include <random>

using namespace std;
using namespace unitree_model;

double get_random()
{
    static std::default_random_engine e;
    e.seed(std::chrono::system_clock::now().time_since_epoch().count()); // seed
    static std::uniform_real_distribution<> dis(0, 1); // rage 0 - 1
    return dis(e);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go1_test_jump2");

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

    // test some random numbers 
    std::cout << "random num " << get_random() << std::endl;
    std::cout << "random num " << get_random() << std::endl;
    std::cout << "random num " << get_random() << std::endl;

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

    // write out some terrain 
    CubeSpawner cubeSpawner;
    float dim_x = 0.5; 
    float dim_y = 0.5;
    float dim_z = 0.04;

    float x=0.99;
    float y=-0.1;
    float z=0.3;
    float mass = 100;
    int counter=3;
    std::cout << "kg name" << "kg"+std::to_string(counter) << std::endl;
    // 
    Vec3<double> rpy(0,0,-0.3);
    Vec4<double> quat;
    quat << ori::rpyToQuat(rpy); // << std::endl;

    // cubeSpawner.spawnCube("kg"+std::to_string(counter),"world",x,y,z,0,0,0,1,dim_x,dim_y,dim_z,mass,int(counter/5000) % 6);
    // cubeSpawner.spawnCube("kg"+std::to_string(counter),"world",x,y,z,quat[1],quat[2],quat[3],quat[0],dim_x,dim_y,dim_z,mass,int(counter/5000) % 6);


    // start jumping object
    JumpingObj jumpObj;
    jumpObj._controlData = _controlData;
    usleep(3000000);
    jumpObj.initJumpingParams();
    jumpObj.init();
    jumpObj.resetObjectiveParams();
    if (jumpObj.do_BO){
        jumpObj.updateParams();
    }
    
    std::cout << "JumpingObj started" << std::endl;

    // write out
    int full_state_len = 76;
    int cpg_state_len = 3;
    std::ofstream full_traj("/data/bellegar/quadruped_logs/full_state_gazebo_jump.txt");
    std::vector<double> full_state(full_state_len);
    jumpObj.writeOutForceTrajectories("/data/bellegar/quadruped_logs/force_trajectories.txt");
    // std::vector<double> cpg_state(cpg_state_len);

    std::ofstream forces_traj("/data/bellegar/quadruped_logs/ee_forces_gazebo_jump.txt");
    std::vector<double> ee_forces(12);

    legController->updateData();
    stateEstimator->run();

    motion_reset();
    
    std::cout << "move to jump start position" << std::endl;
    moveAllPosition(jumpObj.default_joint_pos, 2000);
    sendServoCmd();
    std::cout << "... in init position" << std::endl;
    usleep(3000000);

    param_reset();

    int num_resets = 0;
    counter = 0;
    while (ros::ok() && counter<10000 ){
        // update current robot state
        // _controlData->_legController->updateData();
        // _controlData->_stateEstimator->run();
        // auto& seResult = _controlData->_stateEstimator->getResult();
        // jumpObj.seResult = seResult;
        // for (int i=0; i<4; i++){
        //     jumpObj.real_foot_forces[i] = lowState.footForce[i];
        // }
        // std::cout << lowState.eeForce[0] << std::endl;
        
        // only reset when stopped moving
        if ( (counter > 4500  
                && ( abs(jumpObj.seResult.vBody[0]) + abs(jumpObj.seResult.vBody[1]) + abs(jumpObj.seResult.vBody[2]) < 0.2)) 
                || counter > 5500 ){
            std::cout << "Counter is " << counter << ", reset: move to jump start position" << std::endl;
            jumpObj.getSendObjective();
            if (jumpObj.do_BO){
                // jumpObj.getSendObjective();
                std::cout << jumpObj.seResult.rpy.transpose() << std::endl; 
                //while() don't have new parameters, sleep
                bool new_params = false;
                while(true && num_resets < 99){
                    ros::param::get("new_params", new_params);
                    // std::cout << "new params yet...? " << new_params << std::endl;
                    if (new_params || !jumpObj.do_BO){
                        break;
                    }
                    loop_rate.sleep();
                }

                ros::param::set("new_params", false);
            }
            
            // motion_reset();
            // moveAllPosition(jumpObj.default_joint_pos, 2000);
            motion_reset(jumpObj.default_joint_pos, 1500);
            jumpObj.runStateEstimation();
            std::cout << jumpObj.seResult.rpy.transpose() << std::endl; 
            if (abs(jumpObj.seResult.rpy[0]) > 0.5 || abs(jumpObj.seResult.rpy[1]) > 0.5){
                std::cout << set_model_state_serv.call(model_state) << std::endl;
            }
            param_reset();
            usleep(1000000);
            jumpObj.runStateEstimation();
            
            if (jumpObj.do_BO){
                jumpObj.updateParams(); 
            }
            jumpObj.resetObjectiveParams();
            jumpObj.init();
            counter = 0;
            
            std::cout << "reset complete" << std::endl;
            num_resets++;
            // if (num_resets > 150)
            //     break;
        }

        jumpObj.runStateEstimation();

        /*
        control logic
        */
        jumpObj.getControl();

        // send command to motors
        sendServoCmd();

        // write out data
        jumpObj.getFullState(full_state);
        for(int j=0; j<full_state_len; j++)
            full_traj << full_state[j] << " ";
        full_traj << "\n";

        jumpObj.getForces(ee_forces);
        for(int j=0; j<12; j++){
            forces_traj << ee_forces[j] << " ";
            // std::cout << ee_forces[j] << " "; 
        }
        forces_traj << "\n";
        // std::cout << "\n";


        loop_rate.sleep();
        counter++;
    }

    std::cout << "finished" << std::endl;
    delete legController;
    delete stateEstimator;
    delete _controlData;

    return 0;
}
