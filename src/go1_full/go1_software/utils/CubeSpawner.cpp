/************************************************************************

Check CPG augmented with DRL (Torch from Isaac-Gym)

************************************************************************/
#include "CubeSpawner.h"

CubeSpawner::CubeSpawner() 
{ 
    std::cout << "[CubeSpawner] object created" << std::endl;
    spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
}

void CubeSpawner::spawnCube(const std::string& name, const std::string& frame_id,
    float x, float y, float z, float qx, float qy, float qz, float qw,
    float width, float height, float depth, float mass, int color)
{
    spawnPrimitive(name, true, frame_id, x, y, z, qx, qy, qz, qw, width, height, depth, mass,color);
}

void CubeSpawner::spawnPrimitive(const std::string& name, const bool doCube,
    const std::string& frame_id,
    float x, float y, float z, float qx, float qy, float qz, float qw,
    float widthOrRadius, float height, float depth, float _mass, int color)
{

    geometry_msgs::Pose pose;
    pose.position.x=x;
    pose.position.y=y;
    pose.position.z=z;
    pose.orientation.x=qx;
    pose.orientation.y=qy;
    pose.orientation.z=qz;
    pose.orientation.w=qw;

    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name=name;

    // just so the variable names are shorter..
    float w=widthOrRadius;
    float h=height;
    float d=depth;

    string gazebo_colors[6] ={ "Gazebo/Red",
                               "Gazebo/Green",
                               "Gazebo/Blue",
                               "Gazebo/Yellow",
                               "Gazebo/Purple",
                               "Gazebo/SkyBlue",
    };

    std::stringstream _s;
    if (doCube)
    {
          _s<<"<box>\
            <size>"<<w<<" "<<h<<" "<<d<<"</size>\
          </box>";
    }else{

          _s<<"<cylinder>\
                <length>"<<h<<"</length>\
                <radius>"<<w<<"</radius>\
            </cylinder>";
    }
    std::string geometryString = _s.str();


    float mass=_mass;
    float mass12=mass/12.0;

    // double mu1=500; //500 for PR2 finger tip. In first experiment had it on 1000000
    // double mu2=mu1;
    double mu1=0.8; //500 for PR2 finger tip. In first experiment had it on 1000000
    double mu2=mu1;
    double kp=10000000; //10000000 for PR2 finger tip
    double kd=1; //100 for rubber? 1 fir OR2 finger tip

    bool do_surface=true;
    bool do_inertia=true;

    std::stringstream s;\
    s<<"<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"<<name<<"'>\
        <static>false</static>\
        <link name='link'>";

    // inertia according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    if (do_inertia) 
    {
        double xx, yy, zz;
        if (doCube)
        {
            xx=mass12*(h*h+d*d);
            yy=mass12*(w*w+d*d);
            zz=mass12*(w*w+h*h);
        }
        else
        {
            xx=mass12*(3*w*w + h*h);
            yy=mass12*(3*w*w + h*h);
            zz=0.5*mass*w*w;
        }
        s<<"<inertial>\
        <mass>"<<mass<<"</mass>\
        <inertia>\
          <ixx>"<<xx<<"</ixx>\
          <ixy>0.0</ixy>\
          <ixz>0.0</ixz>\
          <iyy>"<<yy<<"</iyy>\
          <iyz>0.0</iyz>\
          <izz>"<<zz<<"</izz>\
        </inertia>\
          </inertial>";
    }    
    s<<"<collision name='collision'>\
        <geometry>"<<geometryString;
    s<<"</geometry>";
    if (do_surface)
        s<<"<surface>\
            <friction>\
              <ode>\
            <mu>"<<mu1<<"</mu>\
            <mu2>"<<mu2<<"</mu2>\
            <fdir1>0.000000 0.000000 0.000000</fdir1>\
            <slip1>0.000000</slip1>\
            <slip2>0.000000</slip2>\
              </ode>\
            </friction>\
            <bounce>\
              <restitution_coefficient>0.000000</restitution_coefficient>\
              <threshold>100000.000000</threshold>\
            </bounce>\
            <contact>\
              <ode>\
            <soft_cfm>0.000000</soft_cfm>\
            <soft_erp>0.200000</soft_erp>\
            <kp>"<<kp<<"</kp>\
            <kd>"<<kd<<"</kd>\
            <max_vel>100.000000</max_vel>\
            <min_depth>0.001000</min_depth>\
              </ode>\
            </contact>\
        </surface>";
      s<<"</collision>\
          <visual name='visual'>";
      s<<"<geometry>"<<geometryString;
      s<<"</geometry>\
        <material>\
            <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri> \
                <name>"<<gazebo_colors[color]<<"</name>\
            </script>\
        </material>\
          </visual>\
        </link>\
      </model>\
    </sdf>";
    
    spawn.request.model_xml=s.str();
    spawn.request.robot_namespace="cube_spawner";
    spawn.request.initial_pose=pose;
    spawn.request.reference_frame=frame_id;

    //ROS_INFO("Resulting model: \n %s",s.str().c_str());

    //ROS_INFO("Waiting for service");
    spawn_object.waitForExistence();
    //ROS_INFO("Calling service");

    //std::cout<<spawn.request<<std::endl;

    if (!spawn_object.call(spawn)) {
        ROS_ERROR("Failed to call service %s",SPAWN_OBJECT_TOPIC);
    }
    ROS_INFO("Result: %s, code %u",spawn.response.status_message.c_str(), spawn.response.success);
}

