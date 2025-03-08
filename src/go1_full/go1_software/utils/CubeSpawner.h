/************************************************************************

CubeSpawner class

************************************************************************/
#ifndef CUBESPAWNER_H
#define CUBESPAWNER_H

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>

#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
#include <sstream>

using namespace std;

#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"


class CubeSpawner
{
public:
    CubeSpawner();
    void spawnCube(const std::string& name, const std::string& frame_id,
        float x, float y, float z, float qx, float qy, float qz, float qw,
        float width, float height, float depth, float mass,int color);
    void spawnPrimitive(const std::string& name, const bool doCube,
        const std::string& frame_id,
        float x, float y, float z, float qx, float qy, float qz, float qw,
        float widthOrRadius, float height, float depth, float _mass, int color);

private:
    ros::NodeHandle n;
    ros::ServiceClient spawn_object;

    // dimensions
    float dim_x = 0.15;
    float dim_y = 2;
    float dim_z = 0.02; // change this 
    // location 
    float x=3.075;
    float y=0;
    float z=dim_z;
    std::string name="gap_vis";
    std::string frame_id="world";
    float mass=100;

};


#endif // CUBESPAWNER_H




//drop kg on robot
// if ((3000 + counter) % 6000 == 0){
//     auto& seResult = _controlData->_stateEstimator->getResult();
//     seResult.position[i];

//     // dimensions
//     dim_x = 0.05;
//     dim_y = 0.05;
//     dim_z = 0.02; // change this 
//     // location (on top of robot)
//     x=seResult.position[0]+0.09;
//     y=seResult.position[1];
//     z=seResult.position[2] + 0.2;
//     mass = 3;
//     std::cout << "kg name" << "kg"+std::to_string(counter) << std::endl;
//     spawnCube("kg"+std::to_string(counter),frame_id,x,y,z,0,0,0,1,dim_x,dim_y,dim_z,mass,int(counter/5000) % 6);
// }


