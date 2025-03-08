/************************************************************************

Interface for Human-Robot interactions and jumps. 

************************************************************************/
#ifndef HRI_H
#define HRI_H

// Torch 
// #include <torch/script.h> // One-stop header.
#include <iostream>
// #include <memory>

#include "ros/ros.h"
#include <gazebo_msgs/SpawnModel.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <vector>
#include <string.h>
#include <math.h>
#include <random>
#include <chrono>
#include "../include/body.h"
#include "../include/ControlFSMData.h"
#include "../include/Math/orientation_tools.h"
#include "../include/cppTypes.h"
#include <unsupported/Eigen/MatrixFunctions>

#include <numeric>
#include <iomanip>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace Eigen;


class HRI
{
public:
    HRI(); 
    void getFullState(std::vector<double>& full_state);
    void runStateEstimation();

    // spawning object for throwing
    void spawnObject(double mass);

    /************************** StandUp Helpers  ******************************/
    void setLegControllerCommandsFromTrajIndex(int trajIdx);
    void updateLegControllerCommandsFromRL();
    void computeFullTorquesAndSend();
    bool readTraj(std::string filename);
    void moveToTrajInitPos();

    // oscillators
    void init();
    double get_random();
    void IntegrateOscillators();
    void update_oscillators(Vec4<double>& x,Vec4<double>& y,Vec4<double>& z);
    void setLegControllerCommandsFromOscillators();
    void set_des_foot_pos(Vec4<double>& x,Vec4<double>& y,Vec4<double>& z);
    void set_des_foot_vel(Vec4<double>& x,Vec4<double>& y,Vec4<double>& z);
    // Control 
    StateEstimate seResult;
    ControlFSMData* _controlData;
    double sideSign[4] = {-1, 1, -1, 1};
    double foot_y = 0.09; //0.06;

    bool hardwareFlag = false;
    bool throwing = false;
    /***************** Listener for new handshaking parameters ***********/
    void handshakeCallback(const std_msgs::Float64MultiArray& msg);
    bool updated_handshake_params = false;

    void catchObjCallback(const std_msgs::Float64MultiArray& msg);
    void spawnObjCallback(const std_msgs::Float64 msg);
    double closing_time_counter = 0;
    double time_to_close = 0.;
    int close_flag=false; // keep track of open or closed 
    
    /************************** Three leg motion *************************/
    void prepareMove3Legs();
    void setLegControllerCommandsMove3Legs();
    void handshakeControl();
    std::vector<double> base_motion, t;
    int counter  = 0; 
    int counter2 = 0;
    double nom_x3 = 0;
    double nom_y3 = 0.08;
    double nom_z3 = 0.23;
    Vec3<double> actual_nom_p;
    int NUM_SECONDS = 1;
    int traj_len;

    /************************** Oscillator frequencies etc. *************************/
    // keep for all 4 legs for now 
    MatrixXd X = MatrixXd::Zero(2,4);
    MatrixXd X_prev = MatrixXd::Zero(2,4);
    MatrixXd X_dot = MatrixXd::Zero(2,4);
    MatrixXd d2X = MatrixXd::Zero(1,4);

    // double omega = 0.5 *2*M_PI;
    double omega_swing = 0.3*2*M_PI;
    double omega_stance = 4.5*2*M_PI;
    double a_x = 0;//0.05;
    double a_y = 0.04;
    double a_z = 0; //0.05;
    double nom_z = 0.23; 
    double _a = 50; // amplitude convergence 
    double x_offset = 0.0; //-0.02;
    double y_offset = 0.0;
    double des_vel = 0.0;
    double max_vel = 0.0;

    // double nom_z = 0.12; 
    // double x_offset = 0.23;
    // double y_offset = 0.05;
    // double a_y = 0.09;

    // double nom_z = 0.07; 
    // double x_offset = 0.27;
    // double y_offset = 0.0;
    // double a_y = 0.09;

    double dt = 0.001;
    double mu = 1;

    /************************** Optimal trajectories *************************/
    vector<vector<double>> x_opt; // full traj_state
    int full_opt_N; // trajectory length
    // int opt_state_space = 84;

    /************************** Leg Controller options *************************/
    Mat3<double> kpCartesian, kdCartesian;
    Mat3<double> kpJoint, kdJoint;

    // default pos 
    double three_leg_start[12] = {0.0, M_PI/3, -2*M_PI/3, -0.0, M_PI/3, -2*M_PI/3, 
                                  0.0, M_PI/3, -2*M_PI/3, -0.0, M_PI/3, -2*M_PI/3};
    double orig_joint_pos[12] = { 0., 0.785, -1.57,
                                   0., 0.785, -1.57,
                                   0., 0.785, -1.57,
                                   0., 0.785, -1.57  };
    double default_joint_pos[12] = { 0., 0.785, -1.57,
                                   0., 0.785, -1.57,
                                   0., 0.785, -1.57,
                                   0., 0.785, -1.57  };
    double default_foot_pos[12] = { 0., -0.08, -0.3,
                                    0.,  0.08, -0.3,
                                    0., -0.08, -0.3,
                                    0.,  0.08, -0.3};
    double end_joint_pos[12];
    double nom_front_feet_joint_pos[6];
private:

    ros::NodeHandle n;
    ros::Subscriber handshake_sub, catchobj_sub, spawnobj_sub;

    int BASE_POS_INDEX = 0;     // + np.arange(3)
    int BASE_ORN_INDEX = 3;     // + np.arange(3)
    int BASE_LINVEL_INDEX = 6;  // + np.arange(3)
    int BASE_ANGVEL_INDEX = 9;  // + np.arange(3)
    int JOINT_POS_INDEX = 12;   // + np.arange(12)
    int JOINT_VEL_INDEX = 24;   // + np.arange(12)
    //# indices for Cartesian state data
    int FOOT_POS_INDEX = 36;    // + np.arange(12)
    int FOOT_VEL_INDEX = 48;    // + np.arange(12)
    int FOOT_FORCE_INDEX = 60;  // + np.arange(12)
    int TORQUE_INDEX = 72;      // [NEW]


};

#endif // HRI_H