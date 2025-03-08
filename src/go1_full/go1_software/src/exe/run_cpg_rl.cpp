/************************************************************************

Test CPG-RL policies. The "setup" is standard and can mostly be copied between
different files. You MUST try any control algorithms with the simulated 
sensors in Gazebo BEFORE trying on the hardware. 

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
// #include "../../CPG/JumpingObj.h"
#include "../../DRL/IsaacGymTorchInterface.h"
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
    ros::init(argc, argv, "go1_cpg_rl");

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


    // start CPG-RL object
    IsaacGymTorchInterface pyb_interface_obj;
    pyb_interface_obj._controlData = _controlData;
    usleep(3000000);
    
    std::cout << "pyb_interface_obj started" << std::endl;

    // write out
    int full_state_len = 76;
    int cpg_state_len = 24;
    std::ofstream full_traj("/data/bellegar/quadruped_logs/full_state_gazebo_rl.txt");
    std::vector<double> full_state(full_state_len);
    std::vector<double> cpg_state(cpg_state_len);

    // raw observation specifically 
    int obs_len = pyb_interface_obj.obs_len;
    std::ofstream obs_traj("/data/bellegar/quadruped_logs/gazebo_obs.txt");

    legController->updateData();
    stateEstimator->run();

    motion_reset();
    
    // std::cout << "move to jump start position" << std::endl;
    // moveAllPosition(jumpObj.default_joint_pos, 2000);
    // sendServoCmd();
    // std::cout << "... in init position" << std::endl;
    // usleep(3000000);

    // param_reset();

    int num_resets = 0;
    counter = 0;
    double v[3] = {0.4, 0., 0.};
    while (ros::ok() && counter<25000 ){
        // update current robot state
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        auto& seResult = _controlData->_stateEstimator->getResult();
        // jumpObj.seResult = seResult;
        
        /*
        control logic
        */
        if (counter > 8000){
            // double v[3] = {0.5,0.,0.};
            v[0] = 0.5;
            pyb_interface_obj.setCommand(v);
            pyb_interface_obj.cpg.SetGait(4); // change to pace
        } 
        if (counter > 14000){
            // double v[3] = {0.8,0.,0.};
            v[0] = 1.2; //0.8; 
            pyb_interface_obj.setCommand(v);
            pyb_interface_obj.cpg.SetGait(5); // change to pronk
        } 
        if (counter > 19000){
            // double v[3] = {1.0,0.,0.};
            v[0] = 1.0;
            pyb_interface_obj.setCommand(v);
            pyb_interface_obj.cpg.SetGait(7); // change to gallop
        } 
        v[2] = clip(-seResult.rpy[2], -0.2, 0.2);
        // if (counter > 10000){
        //     double v[3] = {0.2,0.,0.};
        //     pyb_interface_obj.setCommand(v);
        // } 
        // query NN for action 
        if (counter % 10 == 0){ // choose new high level command every X*dt s 
        	pyb_interface_obj.computeActionCPG();

            for (int j=0; j < obs_len; j++)
                obs_traj << pyb_interface_obj.rl_obs_data[0][j].item<float>() << " ";
            for (int j=0; j < 12; j++)
                obs_traj << lowState.motorState[j].tauEst << " ";
            obs_traj << "\n";
        }
        // if (counter % 5 == 0){
            pyb_interface_obj.generalSetLegCommandsRLCPG();
            // lowState_pub.publish(lowState);
            sendServoCmd();
        // }

        // write out data
        pyb_interface_obj.getFullState(full_state);
        pyb_interface_obj.getCPGState(cpg_state);
        for(int j=0; j<full_state_len; j++)
            full_traj << full_state[j] << " ";
        for(int j=0; j<cpg_state_len; j++)
            full_traj << cpg_state[j] << " ";
        full_traj << "\n";
        
        loop_rate.sleep();
        counter++;
    }

    std::cout << "finished" << std::endl;
    delete legController;
    delete stateEstimator;
    delete _controlData;

    return 0;
}
