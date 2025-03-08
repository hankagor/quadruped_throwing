/************************************************************************

3-leg test and wave 

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

    motion_reset();
    
    std::cout << "move to start position" << std::endl;
    double thigh = M_PI/3;
    double calf = -2*thigh;
    double pos[12] = {0.0, thigh, calf, -0.0, thigh, calf, 
                      0.0, thigh, calf, -0.0, thigh, calf};
    moveAllPosition(pos, 5000);
    sendServoCmd();
    std::cout << "... in init position" << std::endl;
    usleep(3000000);

    param_reset();

    //cpg related vectors
    Vec4<double> x_out, z_out;
    double sideSign[4] = {-1, 1, -1, 1};
    double foot_y = 0.08;
    double hiplen = 0.08; //0.08; //0.04675; // 0.08 +
    double hiplen2 = 0.04675;
    double nom_z = 0.25; //0.22;

    Mat3<double> kpJoint = Mat3<double>::Zero(); 
    Mat3<double> kdJoint = Mat3<double>::Zero(); 
    kpJoint.diagonal() << 200, 200, 200;
    kdJoint.diagonal() << 3, 3, 3;
    kpJoint.diagonal() << 55, 55, 55;
    kdJoint.diagonal() << 0.8, 0.8, 0.8;

    bool ADD_CARTESIAN_PD = false;
    Mat3<double> kpCartesian = Mat3<double>::Zero(); 
    Mat3<double> kdCartesian = Mat3<double>::Zero(); 
    kpCartesian.diagonal() << 500, 500, 500;
    kdCartesian.diagonal() << 10, 10, 10;
    
    

    // write out
    // int full_state_len = 76; // however many states you would like 
    // std::ofstream full_traj("full_state_gazebo_cpg.txt");
    // std::vector<double> full_state(full_state_len);

    // base motion vector  
    int NUM_SECONDS = 1;
    int traj_len = NUM_SECONDS*1000 + 1;
    std::vector<double> t(traj_len);
    for (int i=0; i<traj_len; i++){
        t[i] = 0.001*i;
    }
    double f = 0.2;        // frequency 
    double max_rpy = -0.7; //0.05;   // max roll, pitch, or yaw 

    // base motion 
    std::vector<double> base_motion(traj_len);
    for (int i=0; i<traj_len; i++){
        base_motion[i] = max_rpy * std::sin(2*M_PI * f * t[i]);
        std::cout << base_motion[i] << std::endl;
    }

    std::vector<double> foot_motion_x(traj_len);
    std::vector<double> foot_motion_z(traj_len);
    for (int i=0; i<traj_len; i++){
        foot_motion_x[i] = 0.1881 + 0.01* std::sin(2*M_PI * f * t[i]);
        foot_motion_z[i] = -nom_z  +0.1881 +  0.01 * std::sin(2*M_PI * f * t[i]);
        std::cout << base_motion[i] << std::endl;
    }

    /*
    issue is that end of one is not aligned with start of other. 
    need to first move foot back to original position
    tooo much scripting ....
    */

    bool start_next=false;

    int counter  = 0;
    int counter2 = 0;
    while (ros::ok() && counter<15000 ){
        // update current robot state
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        auto& seResult = _controlData->_stateEstimator->getResult();

        /*
        control logic
        */
        std::cout << traj_len << " " <<  counter2 << " " 
                  << counter << " rpy " 
                  << seResult.rpy[0] << " " << seResult.rpy[1]  << " " << seResult.rpy[2]  << std::endl;

        std::cout <<  0.1881*std::sin(base_motion[counter % traj_len])  << std::endl;

        // test roll 
        // set joints 
        for (int i=0; i<4; i++){
            // desired foot position and corresponding joint angles
            Vec3<double> pDes, qDes, tau;
            pDes << 0, sideSign[i] * foot_y , -nom_z; 

            if (i==0 || i == 1){ // i==0 || 
                // pDes << 0.1881*std::sin(base_motion[counter % traj_len]), 
                pDes << -0.08*std::sin(base_motion[counter % traj_len]), 
                        sideSign[i] * foot_y , 
                        -nom_z + 0.1881*std::sin(base_motion[counter % traj_len]);
            }
            else if (i==2 || i == 3) {
                // pDes << 0.1881*std::sin(base_motion[counter % traj_len]), 
                pDes << -0.08*std::sin(base_motion[counter % traj_len]), 
                    sideSign[i] * foot_y , 
                    -nom_z ; //- 0.1881*std::sin(base_motion[counter % traj_len]);
            }
        
            std::cout << "des pitch: " << base_motion[counter % traj_len] 
                        << " actual " << seResult.rpy[1] << std::endl;
            

            if (i==0 && counter >= traj_len-1){
                // start trajectory for front foot 
                pDes[0] += min(counter2,1000)*0.0002; 
                pDes[2] += min(counter2,1000)*0.0002;  
            }

            // if (i==0 && counter >= traj_len-1){
            //     // start trajectory for front foot 
            //     pDes << foot_motion_x[counter2 % traj_len], 
            //             sideSign[i] * foot_y , 
            //             foot_motion_z[counter2 % traj_len];
            // }
            

            computeInverseKinematics(_controlData->_legController->_quadruped, pDes, i, &qDes);
            for (int j = 0; j < 3; j++){
                // test joint PD control
                lowCmd.motorCmd[i*3+j].q = qDes[j];
                lowCmd.motorCmd[i*3+j].dq = 0;
                lowCmd.motorCmd[i*3+j].tau = 0;
                lowCmd.motorCmd[i*3+j].Kp = kpJoint(j,j);
                lowCmd.motorCmd[i*3+j].Kd = kdJoint(j,j);
                _controlData->_legController->commands[i].vDes[j] = 0; 

            }
        }
        

        // send command to motors
        sendServoCmd();

        loop_rate.sleep();
        if (counter < traj_len -1){
            counter++;
        }
        else if (counter2 < traj_len -1){
            counter2++;
        }
    }

    std::cout << "finished" << std::endl;
    delete legController;
    delete stateEstimator;
    delete _controlData;

    return 0;
}
