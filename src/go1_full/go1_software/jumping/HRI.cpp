/************************************************************************

Interface for Human-Robot Interactions and jumps. 

************************************************************************/

#include "HRI.h"

HRI::HRI() // : map( { "elevation" })
{
    std::cout << "[HRI] init object" << std::endl;
    handshake_sub = n.subscribe("/handshake_params", 1, &HRI::handshakeCallback, this);
    catchobj_sub  = n.subscribe("/catchobj_params", 1, &HRI::catchObjCallback, this);
    spawnobj_sub  = n.subscribe("/spawnobj_cmd", 1, &HRI::spawnObjCallback, this);
}

#include <vector>
#include <sstream>
#include <string>

std::string generateBox(std::vector<double> pose, std::vector<double> size, double mass = 1.0) {
    double Ixx = (1.0/12.0) * mass * (size[1] * size[1] + size[2] * size[2]);
    double Iyy = (1.0/12.0) * mass * (size[0] * size[0] + size[2] * size[2]);
    double Izz = (1.0/12.0) * mass * (size[0] * size[0] + size[1] * size[1]);

    std::stringstream ss;
    ss << "<?xml version=\"1.0\" ?>\n"
       << "<sdf version=\"1.6\">\n"
       << "  <model name=\"small_box\">\n"
       << "    <static>false</static>\n"
       << "    <link name=\"link\">\n"
       << "      <pose>" << pose[0] << " " << pose[1] << " " << pose[2] << " 0 -0 0</pose>\n"
       << "      <inertial>\n"
       << "        <mass>" << mass << "</mass>\n"
       << "        <inertia>\n"
       << "          <ixx>" << Ixx << "</ixx>\n"
       << "          <iyy>" << Iyy << "</iyy>\n"
       << "          <izz>" << Izz << "</izz>\n"
       << "          <ixy>0.0</ixy>\n"
       << "          <ixz>0.0</ixz>\n"
       << "          <iyz>0.0</iyz>\n"
       << "        </inertia>\n"
       << "      </inertial>\n"
       << "      <visual name=\"visual\">\n"
       << "        <geometry>\n"
       << "          <box>\n"
       << "            <size>" << size[0] << " " << size[1] << " " << size[2] << "</size>\n"
       << "          </box>\n"
       << "        </geometry>\n"
       << "      </visual>\n"
       << "      <collision name=\"collision\">\n"
       << "        <geometry>\n"
       << "          <box>\n"
       << "            <size>" << size[0] << " " << size[1] << " " << size[2] << "</size>\n"
       << "          </box>\n"
       << "        </geometry>\n"
       << "        <surface>\n"
       << "          <friction>\n"
       << "            <ode>\n"
       << "              <mu>10</mu>\n"
       << "              <mu2>20</mu2>\n"
       << "            </ode>\n"
       << "          </friction>\n"
       << "          <bounce>\n"
       << "            <restitution_coefficient>0.2</restitution_coefficient>\n"
       << "          </bounce>\n"
       << "        </surface>\n"
       << "      </collision>\n"
       << "    </link>\n"
       << "  </model>\n"
       << "</sdf>";
    
    return ss.str();
}




void HRI::spawnObject(double mass) {
    vector<double> pose(3);
    vector<double> size(3);
    
    _controlData->_legController->updateData();
    _controlData->_stateEstimator->run();
    seResult = _controlData->_stateEstimator->getResult();
    
    for (int i = 0; i < 2; i++)
        pose[i] = -(_controlData->_legController->data[0].p[i] + 
                        _controlData->_legController->data[1].p[i]) / 2.0 + seResult.position[i];
    pose[2] = -(_controlData->_legController->data[0].p[2] + 
                        _controlData->_legController->data[1].p[2]) / 2.0;
    // pose[2] += 0.18;
    pose[0] -= 0.05;
    pose[2] += 0.01;
    size[0] = 0.15; size[2] = 0.15;
    size[1] = (_controlData->_legController->data[1].p[1] -
                    _controlData->_legController->data[0].p[1]) + 0.08;
    ROS_INFO("size %.4f", size[1]);
    ROS_INFO("body %.4f x %.4f x %.4f", seResult.position[0], seResult.position[1], seResult.position[2]);
    
    ROS_INFO("left leg %.4f x %.4f x %.4f", _controlData->_legController->data[1].p[0], _controlData->_legController->data[1].p[1], _controlData->_legController->data[1].p[2]);
    ROS_INFO("left leg %.4f x %.4f x %.4f", _controlData->_legController->data[0].p[0], _controlData->_legController->data[0].p[1], _controlData->_legController->data[0].p[2]);
    ROS_INFO("left leg %.4f x %.4f x %.4f", _controlData->_legController->data[2].p[0], _controlData->_legController->data[2].p[1], _controlData->_legController->data[2].p[2]);
    ROS_INFO("left leg %.4f x %.4f x %.4f", _controlData->_legController->data[3].p[0], _controlData->_legController->data[3].p[1], _controlData->_legController->data[3].p[2]);
    
    ros::ServiceClient spawn_client = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    
    std::string box_sdf = generateBox(pose, size, mass);

    
    gazebo_msgs::SpawnModel spawn_srv;
    spawn_srv.request.model_name = "my_box";
    spawn_srv.request.model_xml = box_sdf; 
    spawn_srv.request.robot_namespace = "";


    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Successfully spawned box with size %.2f x %.2f x %.2f", size[0], size[1], size[2]);
        ROS_INFO("Successfully spawned box with pose %.2f x %.2f x %.2f", pose[0], pose[1], pose[2]);
    } else {
        ROS_ERROR("Failed to spawn the box.");
    }
}
/**
 * Expect the following data convention order 
    amplitude a_x
    amplitude a_z
    frequency omega_swing
    frequency omega_stance 
    kpCartesian 
    kdCartesian 

    Total: 6 values 
 */
void HRI::handshakeCallback(const std_msgs::Float64MultiArray& msg){
    // [TODO] make sure these are in reasonable ranges 
    a_x = msg.data[0]; 
    a_z = msg.data[1];
    omega_swing =  msg.data[2]*2*M_PI;
    omega_stance = msg.data[3]*2*M_PI;
    kpCartesian.diagonal() << msg.data[4], msg.data[4], msg.data[4];
    kdCartesian.diagonal() << msg.data[5], msg.data[5], msg.data[5];

    updated_handshake_params = true;
}

/**
 * Expect the following data convention order 
    time_to_close (s)
    x pos in robot frame 
    y pos in robot frame
    z pos in robot frame  

    Total: 4 values 
 */
void HRI::catchObjCallback(const std_msgs::Float64MultiArray& msg){
    // [TODO] make sure these are in reasonable ranges 
    time_to_close = msg.data[0]; 
    x_offset = msg.data[1];
    y_offset = msg.data[2];
    nom_z = msg.data[3];
    if (time_to_close > 0.5)
        throwing = 1;
    else
        throwing = 0;
    // x_offset = clip(x_offset, -0.1, 0.1);
    // y_offset = clip(y_offset)
    if (msg.data.size() >= 5){
        close_flag = msg.data[4];
    }
    if (msg.data.size() >= 6)
        des_vel = msg.data[5];
    if (msg.data.size() >= 7)
        max_vel = msg.data[6];
    // updated_handshake_params = true;
    // close_flag = !close_flag;
    // std::cout << "Received params to catch and set open to: " << close_flag << std::endl;
}

void HRI::spawnObjCallback(const std_msgs::Float64 msg){
    std::cout << "Spawning..." << std::endl;
    if (msg.data)
        HRI::spawnObject(msg.data);
    
}
/****************************************************************************************************************************
**************************************** Get state robot state (and run estimation) *****************************************
*****************************************************************************************************************************/

/**
 * Place full state in vector
 * Body: pos xyz
             rpy
             lin vel
             ang vel
        joint pos
            vel
            torques
        foot pos
             vel
        contact booleans 
 */
void HRI::getFullState(std::vector<double>& full_state){
    // data
    seResult = _controlData->_stateEstimator->getResult();
    // idx
    int full_idx = 0;
    // body pos
    for (int i = 0; i < 3; i++){
        full_state[full_idx+i] = seResult.position[i];
    }
    full_idx += 3;
    // body rpy
    for (int i = 0; i < 3; i++){
        full_state[full_idx+i] = seResult.rpy[i];
    }
    // std::cout << "quaternion: " << seResult.orientation[0] << " " << seResult.orientation[1] << " " << seResult.orientation[2]
    //                             << " " << seResult.orientation[3] << std::endl;
    full_idx += 3;
    // body frame linear velocity 
    for (int i = 0; i < 3; i++){
        full_state[full_idx+i] = seResult.vBody[i];
    }
    full_idx += 3;
    // body frame angular velocity
    for (int i = 0; i < 3; i++){
        full_state[full_idx+i] = seResult.omegaBody[i];
    }
    full_idx += 3;
    for(int i = 0; i < 12; i++){
        full_state[full_idx+i] = lowState.motorState[i].q;
        // std::cout << lowState.motorState[i].q << std::endl;
        full_state[full_idx+i+12] = lowState.motorState[i].dq;
        
        full_state[full_idx+i+24] = lowState.motorState[i].tauEst;
    }
    full_idx += 36;
    // foot positions and velocities 
    for (int leg=0; leg<4; leg++){
        // foot pos
        for(int j=0; j<3; j++){
            full_state[full_idx+j] = _controlData->_legController->data[leg].p[j];
        }
        full_idx+=3;
        // foot vel
        for(int j=0; j<3; j++){
            full_state[full_idx+j] = _controlData->_legController->data[leg].v[j];
        }
        // if (leg == 0) std::cout << full_state[full_idx] << std::endl;; 
        full_idx+=3;
    }
    // contact bools
    //std::cout << "==" << std::endl;
    for (int i = 0; i < 4; i++){
        // if (lowState.footForce[i] > 0){
        //     full_state[idx+i] = 1; // positive force, so in contact
        // }
        // else{
        //     full_state[idx+i] = 0; // in air
        // }
        full_state[full_idx+i] = lowState.footForce[i];
        //std::cout << _controlData->_legController->commands[i].feedforwardForce << std::endl;
    }
}

void HRI::runStateEstimation(){
    _controlData->_legController->updateData();
    _controlData->_stateEstimator->run();
    seResult = _controlData->_stateEstimator->getResult();
    // std::cout << lowState.motorState[2].q << " " << lowState.motorState[2].dq << std::endl ;
    // std::cout << lowState.motorState[5].q << " " << lowState.motorState[5].dq << std::endl ;
    // for (int leg=0; leg<2; leg++){
    //     if (_controlData->_legController->data[leg].v[0] < 0.5)
    //         continue;
    //     // foot pos
    //     for(int j=0; j<3; j++){
    //         std::cout << _controlData->_legController->data[leg].p[j] << " ";
    //     }
    //     std::cout << "velocity: ";
    //     for(int j=0; j<3; j++){
    //         std::cout << _controlData->_legController->data[leg].v[j] << " ";
    //     }
    //     std::cout << std::endl;
    //     std::cout << std::endl;
    // }
    // _controlData->_legController->publishPosCmd();
}


/****************************************************************************************************************************
***************************************************** three leg *************************************************************
*****************************************************************************************************************************/
void HRI::prepareMove3Legs()
{
    traj_len = NUM_SECONDS*1000 + 1;
    t.resize(traj_len);
    for (int i=0; i<traj_len; i++){
        t[i] = 0.001*i;
    }
    double f = 0.2;         // frequency 
    // double max_rpy = -0.7;  // max roll, pitch, or yaw 
    double max_rpy = -1;  // max roll, pitch, or yaw 

    // base motion 
    base_motion.resize(traj_len);
    for (int i=0; i<traj_len; i++){
        base_motion[i] = max_rpy * std::sin(2*M_PI * f * t[i]);
    }
    counter = 0;
    counter2 = 0;
}

void HRI::setLegControllerCommandsMove3Legs()
{
    // kpJoint.diagonal() << 55, 55, 55;
    // kdJoint.diagonal() << 0.8, 0.8, 0.8;
    bool do_hand_shaking = false;

    // set joints 
    for (int i=0; i<4; i++){
        // desired foot position and corresponding joint angles
        Vec3<double> pDes, qDes; //, tau;
        pDes << 0, sideSign[i] * nom_y3 , -nom_z3; 

        // front legs
        if (i==0 || i == 1){ 
            pDes << -nom_y3*std::sin(base_motion[counter % traj_len]), 
                    sideSign[i] * (nom_y3 + 0.07*std::sin(base_motion[counter % traj_len])), 
                    -nom_z3 + 0.1881*std::sin(base_motion[counter % traj_len]);
        }
        // rear legs 
        else if (i==2 || i == 3) {
            // pDes << -nom_y3*std::sin(base_motion[counter % traj_len]), 
            //         sideSign[i] * nom_y3 , 
            //         -nom_z3; // - 0.2*std::sin(base_motion[counter % traj_len]); 
            pDes << -nom_y3*std::sin(base_motion[counter % traj_len]), 
                    sideSign[i] * nom_y3 , 
                    -nom_z3; //- 0.03*std::sin(base_motion[counter % traj_len]); 
        }
        
        // move front right leg up
        if (i==0 && counter >= traj_len-1){
            if(counter2 < traj_len -1){
                pDes[0] += min(counter2,1000)*0.0002; 
                pDes[2] += min(counter2,1000)*0.0002;  
                // record foot pos (to be used as nominal offset for handshake)
                computeLegJacobianAndPosition(_controlData->_legController->_quadruped, 
                                                _controlData->_legController->data[i].q,
                                                &(_controlData->_legController->data[i].J),
                                                &actual_nom_p,i);
            }
            else{
                // start shaking 
                do_hand_shaking = true;
            }
        }

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

    // NOTE: this will over-write the front right foot commands
    if (do_hand_shaking){
        handshakeControl();
    }
    _controlData->_legController->updateCommandNoSend(); 

    if (counter < traj_len -1){
        counter++;
    }
    else if (counter2 < traj_len -1){
        counter2++;
    }

}
/*
 * Based on CPG, update front right foot position.
 *  [TODO]: we want to learn/update: 
                -amplitudes (i.e. a_x, a_z)
                -frequencies (i.e. omega_swing, omega_stance) 
                -gains (kpCartesian, kdCartesian)
        with preference learning. 

    [TODO]: after some "handshaking" time, we want to compute features and send back
            to preference learning code. We already know the "features", so this could
            also just happen in python exclusively, unless we need to know things like torque. 
 */
void HRI::handshakeControl(){
    
    IntegrateOscillators();

    // only doing for one leg (Front Right)! 
    int leg = 0;

    // kpCartesian.diagonal() << 400, 400, 400;
    // kdCartesian.diagonal() << 8, 8, 8;

    // set handshake amplitudes (in task space)
    //               frequencies (swing/stance notation left from locomotion)
    //               Cartesian gains 
    // [TODO]: these variables should be updated elsewhere in a function with a ROS message 
    //         coming from the preference learning code 
    if (!updated_handshake_params){
        a_x = 0.05; 
        a_z = 0.05;
        omega_swing =  2.0*2*M_PI;
        omega_stance = 2.0*2*M_PI;
        kpCartesian.diagonal() << 100, 100, 100;
        kdCartesian.diagonal() << 2, 2, 2;
    }

    // get desired foot x and z position in leg grame 
    double x = actual_nom_p[0] + a_x * std::sin(X(1,leg));
    double z = actual_nom_p[2] + a_z * std::sin(X(1,leg));
    Vec3<double> pDes, qDes, tau; 
    pDes << x, nom_y3*sideSign[leg], z;
    // computeInverseKinematics(_controlData->_legController->_quadruped, pDes, leg, &qDes);

    for (int j=0; j<3; j++){
        // since we are doing force control, set desired joint positions and gains to 0.
        // NOTE: it is good to have some damping (Kd) for the hardware 
        lowCmd.motorCmd[leg*3+j].q  = PosStopF; 
        lowCmd.motorCmd[leg*3+j].dq = 0;
        lowCmd.motorCmd[leg*3+j].Kp = 0;
        lowCmd.motorCmd[leg*3+j].Kd = 2; 
        lowCmd.motorCmd[leg*3+j].tau = 0; 

        // for Cartesian PD. Note: torques are computed in LegController
        _controlData->_legController->commands[leg].pDes[j] = pDes[j];
        _controlData->_legController->commands[leg].vDes[j] = 0;
        _controlData->_legController->commands[leg].kpCartesian(j,j) = kpCartesian(j,j);
        _controlData->_legController->commands[leg].kdCartesian(j,j) = kdCartesian(j,j);
        _controlData->_legController->commands[leg].feedforwardForce[j]  = 0;
    }        
    // compute torque in LegController (Cartesian PD / impedance control depending on feedforward force
    _controlData->_legController->updateCommandNoSend();

}

/****************************************************************************************************************************
******************************************************* stand up ************************************************************
*****************************************************************************************************************************/

/*
 * Given traj index, get the associated control commands to track trajectory 
 */
void HRI::setLegControllerCommandsFromTrajIndex(int trajIdx)
{
    // kpJoint.diagonal() << 50, 50, 50;
    // kdJoint.diagonal() << 1, 1, 1;
    // kpJoint.diagonal() << 60, 60, 60;
    // kdJoint.diagonal() << 1, 1, 1;

    auto des_state = x_opt[trajIdx];
    for(int leg=0; leg<4; leg++){
        Vec3<double> qDes, qdDes, tau, pDes, vDes; 
        for (int j=0; j<3; j++){
            qDes[j] = des_state[JOINT_POS_INDEX+leg*3+j];
            qdDes[j] = des_state[JOINT_VEL_INDEX+leg*3+j];
            tau[j] = des_state[TORQUE_INDEX+leg*3+j];
            pDes[j] = des_state[FOOT_POS_INDEX+leg*3+j];
            vDes[j] = des_state[FOOT_VEL_INDEX+leg*3+j];

            // lowCmd
            lowCmd.motorCmd[leg*3+j].q = qDes[j];
            lowCmd.motorCmd[leg*3+j].dq = qdDes[j];
            lowCmd.motorCmd[leg*3+j].Kd = kdJoint(j,j);
            lowCmd.motorCmd[leg*3+j].Kp = kpJoint(j,j);
            lowCmd.motorCmd[leg*3+j].tau = 0; //tau[j]; // doesn't work well with feedforward torque (should be 2x or 0.5x (?))

            // for Cartesian PD as well (not needed I guess)
            // _controlData->_legController->commands[leg].pDes[j] = pDes[j];
            // _controlData->_legController->commands[leg].vDes[j] = vDes[j];
            // _controlData->_legController->commands[leg].kpCartesian(j,j) = kpCartesian(j,j);
            // _controlData->_legController->commands[leg].kdCartesian(j,j) = kdCartesian(j,j);
            // _controlData->_legController->commands[leg].feedforwardForce[j] = force_i[j];

            _controlData->_legController->commands[leg].pDes[j] = 0;
            _controlData->_legController->commands[leg].vDes[j] = 0;
            _controlData->_legController->commands[leg].kpCartesian(j,j) = 0;
            _controlData->_legController->commands[leg].kdCartesian(j,j) = 0;
            _controlData->_legController->commands[leg].feedforwardForce[j] = 0;

        }
        
    }
    // _controlData->_legController->updateCommandWithTauNoSend(); 
    _controlData->_legController->updateCommandNoSend(); 

}

void HRI::moveToTrajInitPos(){
    std::cout << "[moveToTrajInitPos] reset Go1 to starting pos..... " << std::endl;
    for (int i=0; i<12; i++){
        lowCmd.motorCmd[i].Kp = 80;
        lowCmd.motorCmd[i].Kd = 2;
    }
    moveAllPosition(orig_joint_pos, 1000);
    std::cout << "... done " << std::endl;
}

void HRI::init(){
    std::cout << "[HRI] init " << std::endl;
    X.setZero();
    X_dot.setZero();
    // init CPG amplitude and phase
    for(int i=0; i<4; i++){
        X(0,i) = get_random() * 0.1;
        X(1,i) = 0; //PHI(0,i);
    }

    // init gains 
    kpCartesian << 500, 0, 0,
                    0, 500, 0,
                    0, 0, 500; //100
    kdCartesian << 10, 0, 0,
                    0, 10, 0,
                    0, 0, 10;
    kpJoint << 100, 0, 0,
                0, 100, 0,
                0, 0, 100; //100
    kdJoint<<   2, 0, 0,
                0, 2, 0,
                0, 0, 2;

    for(int leg=0; leg<2; leg++){
        Vec3<double> pDes, qDes; //, tau; 
        pDes << 0, sideSign[leg]*foot_y, -nom_z;
        computeInverseKinematics(_controlData->_legController->_quadruped, pDes, leg, &qDes);
        for (int j=0; j<3; j++){
            nom_front_feet_joint_pos[leg*3+j] = qDes[j];
        }
    }
    prepareMove3Legs();
    std::cout << "[HRI] init done" << std::endl;
}

/*
 * Helper function to get random variables 
 */
double HRI::get_random()
{
    static std::default_random_engine e;
    e.seed(std::chrono::system_clock::now().time_since_epoch().count()); // seed
    static std::uniform_real_distribution<> dis(0, 1); // rage 0 - 1
    return dis(e);
}

/*
 * Integrate oscillators: update amplitudes and phases  
 */
void HRI::IntegrateOscillators()
{
    MatrixXd X_copy, X_dot_prev;
    X_copy = X.replicate(1,1);
    X_dot_prev = X_dot.replicate(1,1);
    X_dot.setZero();

    double r, phi, omega;
    for(int i=0; i<4; i++){
        omega = 0;
        r   = X(0,i);
        phi = X(1,i);
        // amplitude
        double R_dot = _a * (mu - pow(r,2)) * r;
        // phase
        phi = std::fmod(phi, 2*M_PI);
        if (phi > M_PI){
            omega = omega_stance;
        }
        else{
            omega = omega_swing;
        }
        X_dot.col(i) << R_dot, omega;
    }

    // integrate 
    X = X + (X_dot_prev + X_dot) * dt / 2;
    for (int i=0; i<4; i++){
        X(1,i) = fmod(X(1,i),2*M_PI);
    }
} 

/*
 * Based on oscillator states, get corresponding foot positions   
 */
void HRI::update_oscillators(Vec4<double>& x,Vec4<double>& y,Vec4<double>& z){
    IntegrateOscillators();

    // x
    for (int i=0; i<4 ; i++){
        x[i] = x_offset + a_x * std::cos(X(1,i));
    }
    // y
    for (int i=0; i<4 ; i++){
        if (i==0 || i ==2){
            y[i] = y_offset + sideSign[i] * foot_y +  a_y * std::cos(X(1,i));
        }
        else{
            y[i] = y_offset + sideSign[i] * foot_y -  a_y * std::cos(X(1,i));
        }  
    }
    // z
    for(int i=0; i<4; i++){
        z[i] = -nom_z + a_z * std::sin(X(1,i));
    }
}

/*
 * get desired foot positions (sent from elsewhere)   
 */
void HRI::set_des_foot_pos(Vec4<double>& x,Vec4<double>& y,Vec4<double>& z){
    // x
    for (int i=0; i<4 ; i++){
        x[i] = x_offset + a_x;
    }
    // y
    for (int i=0; i<4 ; i++){
        if (i==0 || i ==2){
            y[i] = y_offset + sideSign[i] * foot_y +  (a_y * close_flag);
        }
        else{
            y[i] = y_offset + sideSign[i] * foot_y -  (a_y * close_flag);
        }  
    }
    // z
    for(int i=0; i<4; i++){
        z[i] = -nom_z + a_z;
    }
    // a_y += 1e-6;
}

void HRI::set_des_foot_vel(Vec4<double>& x,Vec4<double>& y,Vec4<double>& z){
    // x
    for (int i=0; i<4 ; i++){
        x[i] = des_vel * sqrt(2)/2;
    }
    // y
    for (int i=0; i<4 ; i++){
        if (i==0 || i ==2){
            y[i] = 0;
        }
        else{
            y[i] = 0;
        }  
    }
    // z
    for(int i=0; i<4; i++){
        // z[i] = 0;
        z[i] = -des_vel * sqrt(2)/2;
    }
}

void HRI::setLegControllerCommandsFromOscillators(){
    Vec4<double> x_out, y_out, z_out;
    Vec4<double> x_vel, y_vel, z_vel;
    // update_oscillators(x_out,y_out,z_out);
    set_des_foot_pos(x_out, y_out, z_out);
    set_des_foot_vel(x_vel, y_vel, z_vel);
    // des_vel = 0.0;

    kpCartesian.diagonal() << 600, 600, 600;
    kdCartesian.diagonal() << 11, 11, 11;

    // kpCartesian.diagonal() << 500, 500, 500;
    // kdCartesian.diagonal() << 10, 10, 10;

    // only update commands for front legs (which are in the air)
    for(int leg=0; leg<2; leg++){
        Vec3<double> pDes, vDes, qDes, tau; 
        pDes << x_out[leg], y_out[leg], z_out[leg];
        vDes << x_vel[leg], y_vel[leg], z_vel[leg];
        computeInverseKinematics(_controlData->_legController->_quadruped, pDes, leg, &qDes);

        for (int j=0; j<3; j++){
            // lowCmd
            lowCmd.motorCmd[leg*3+j].q  = PosStopF; 
            lowCmd.motorCmd[leg*3+j].dq = 0;
            lowCmd.motorCmd[leg*3+j].Kp = 0;
            lowCmd.motorCmd[leg*3+j].Kd = 2; 
            lowCmd.motorCmd[leg*3+j].tau = 0; 

            // for Cartesian PD as well 
            _controlData->_legController->commands[leg].pDes[j] = pDes[j];
            _controlData->_legController->commands[leg].vDes[j] = vDes[j];
            _controlData->_legController->commands[leg].kpCartesian(j,j) = kpCartesian(j,j);
            _controlData->_legController->commands[leg].kdCartesian(j,j) = kdCartesian(j,j);
            _controlData->_legController->commands[leg].feedforwardForce[j]  = 0;
        }        
    }
    _controlData->_legController->updateCommandNoSend(); 
    
}

/****************************************************************************************************************************
*********************************************** loading/sending *******************************************************
*****************************************************************************************************************************/


/*
 * Load optimal traj from MATLAB
 * Read the trajectory file, this assumes it is ROW order, i.e. row 0 is the first full state (x), row 1 is (z), etc.
 */
bool HRI::readTraj(std::string filename){

    std::ifstream trajFile;
    trajFile.open(filename); 
    // If cannot open the file, report an error
    if(!trajFile.is_open())
    {
        std::cout << "Could not open optimal trajectory file." << std::endl;
        return false;
    }
    cout << "Reading optimal trajectory... " << filename << endl;

    // clear vector in case already loaded some trajectory
    x_opt.clear();
    std::string tempstr;
    double tempdouble;
    full_opt_N = 0;
    int idx = 0;

    while (getline(trajFile, tempstr)) {
        std::istringstream iss(tempstr);
        std::vector<double> tempv;

        while (iss >> tempdouble) {
            tempv.push_back(tempdouble);
            if(iss.peek() == ',') iss.ignore();
        }
        x_opt.push_back(tempv);
        full_opt_N += 1;
        idx += 1;
    }

    cout << "Traj is " << full_opt_N << " knot points. Setting initial joint pos." << endl;
    
    for (int i=0; i<12; i++){
        default_joint_pos[i] = x_opt[0][JOINT_POS_INDEX+i];
        end_joint_pos[i] = x_opt[full_opt_N-1][JOINT_POS_INDEX+i]; 
    }

    trajFile.close();
    cout << "...done." << endl;
    return true;
}