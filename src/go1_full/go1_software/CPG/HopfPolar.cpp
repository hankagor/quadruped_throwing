/************************************************************************

Hopf Polar CPG

************************************************************************/
#include "HopfPolar.h"


HopfPolar::HopfPolar() 
{ 
    std::cout << "[HopfPolar] object created" << std::endl;
}

void HopfPolar::init(){
    std::cout << "[HopfPolar] init function" << std::endl;
    X.setZero();
    X_dot.setZero();
    SetGait(gait);
    // init CPG amplitude and phase
    for(int i=0; i<4; i++){
        X(0,i) = get_random() * 0.1;
        X(1,i) = PHI(0,i);
    }
    std::cout << "[HopfPolar] init done" << std::endl;
}

/*
 * Helper function to get random variables 
 */
double HopfPolar::get_random()
{
    static std::default_random_engine e;
    e.seed(std::chrono::system_clock::now().time_since_epoch().count()); // seed
    static std::uniform_real_distribution<> dis(0, 1); // rage 0 - 1
    return dis(e);
}

/*
Gaits:
1 bounding
2 trotting
3 walking
4 pacing;
*/
void HopfPolar::SetGait(int gaitNum )
{
    if (gaitNum == 1){ // bounding 
        PHI <<   0, 0, 1, 1,
                 0, 0, 1, 1,
                -1,-1, 0, 0,
                -1,-1, 0, 0;
        PHI = -M_PI * PHI;
        K_tegotae << 0, 0, 1, 1,
                     0, 0, 1, 1,
                     1, 1, 0, 0,
                     1, 1, 0, 0; 
    }
    else if (gaitNum == 2){ // trotting 
        PHI <<   0, 1, 1, 0,
                -1, 0, 0,-1,
                -1, 0, 0,-1,
                 0, 1, 1, 0;
        PHI = M_PI * PHI;
        K_tegotae << 0, 1, 1, 0,
                     1, 0, 0, 1,
                     1, 0, 0, 1,
                     0, 1, 1, 0;
    }
    else if (gaitNum == 3){ // walking 
        PHI <<       0,   M_PI,-M_PI/2, M_PI/2,
                  M_PI,      0, M_PI/2,-M_PI/2,
                M_PI/2,-M_PI/2,      0,   M_PI,
               -M_PI/2, M_PI/2,   M_PI,      0;
        PHI = -PHI;
        K_tegotae << 0, 1, 1, 0,
                     1, 0, 0, 1,
                     1, 0, 0, 1,
                     0, 1, 1, 0;
    }
    else if (gaitNum == 4){ // pace 
        PHI <<   0, 1, 0, 1,
                -1, 0,-1, 0,
                 0, 1, 0, 1,
                -1, 0,-1, 0;
        PHI = M_PI * PHI;
        K_tegotae << 0, 1, 0, 1,
                     1, 0, 1, 0,
                     0, 1, 0, 1,
                     1, 0, 1, 0;
    }
    else{
        std::cout << "INVALID GAIT NUMBER" << gaitNum << std::endl; 
    }
}


void HopfPolar::update(Vec4<double>& x_out,Vec4<double>& z_out){
    Integrate();
    Vec4<double> x = Vec4<double>::Zero();
    for (int i=0; i<4 ; i++){
        x[i] = X(0,i) * std::cos(X(1,i));
    }

    Vec4<double> z_temp = Vec4<double>::Zero(); // bookkeepping
    for(int i=0; i<4; i++){
        double phi = X(1,i);
        if (std::sin(phi) > 0){//in swing
            z_temp[i] = -h_max + ground_clearance * std::sin(phi);
        }
        else{
            z_temp[i] = -h_max + ground_penetration * std::sin(phi);
        }
    }

    // return normal indices
    for (int i=0; i<4; i++){
        x_out[i] = -des_step_len * x[LEG_INDICES[i]] + x_offset;
        z_out[i] = z_temp[LEG_INDICES[i]];
    }

}


void HopfPolar::Integrate()
{
    MatrixXd X_copy, X_dot_prev;
    X_copy = X.replicate(1,1);
    X_dot_prev = X_dot.replicate(1,1);
    X_dot.setZero();
    // get foot forces
    // auto Fn = lowState.footForce;
    // for (int i=0; i<4; i++){
    //     Fn[i] = lowState.footForce[LEG_INDICES[i]];
    // }

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
        if (couple){
            //omega += X.row(0).dot(coupling_strength * sin(X.row(1) - phi - PHI.row(i)));
            for (int j=0; j<4; j++){
                omega += X(0,j) * coupling_strength * std::sin(X(1,j) - phi - PHI(i,j));
            }
            // if (tegotae_feedback){
            //     // add normal force feedback (Tegotae)
            //     omega += -0.05 * Fn[i] * std::cos(phi);
            // }
            // if (coupled_tegotae_feedback){
            //     //coupled normal forces
            //     for (int j=0; j<4; j++){
            //         omega += 0.1 * K_tegotae(i,j)*Fn[i];
            //     }
            // }
        }
        X_dot.col(i) << R_dot, omega;
    }

    // integrate 
    X = X + (X_dot_prev + X_dot) * dt / 2;
    for (int i=0; i<4; i++)
        X(1,i) = fmod(X(1,i),2*M_PI);
} 

void HopfPolar::runStateEstimation(){
    _controlData->_legController->updateData();
    _controlData->_stateEstimator->run();
    seResult = _controlData->_stateEstimator->getResult();
}


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
void HopfPolar::getFullState(std::vector<double>& full_state){
    // data
    //auto& seResult = _controlData->_stateEstimator->getResult();
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