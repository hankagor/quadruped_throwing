#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <sstream>

std::string generateBox(vector<double> pose, vector<double> size) {
    std::stringstream ss;
    ss << "<sdf version='1.6'>"
       << "<model name='my_box'>"
       << "  <pose> " << pose[0] << " " << pose[1] << " " << pose[2] << "</pose>"
       << "  <link name='link'>"
       << "    <inertial><mass>1.0</mass></inertial>"
       << "    <collision name='collision'>"
       << "      <geometry>"
       << "        <box><size>" << size[0] << " " << size[1] << " " << size[2] << "</size></box>"
       << "      </geometry>"
       << "    </collision>"
       << "    <visual name='visual'>"
       << "      <geometry>"
       << "        <box><size>" << size[0] << " " << size[1] << " " << size[2] << "</size></box>"
       << "      </geometry>"
       << "    </visual>"
       << "  </link>"
       << "</model>"
       << "</sdf>";
    return ss.str();
}

void spawnObject(ros::NodeHandle nh) {
    
    ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    std::string box_sdf = generateBox(size_x, size_y, size_z);

    
    gazebo_msgs::SpawnModel spawn_srv;
    spawn_srv.request.model_name = "my_box";
    spawn_srv.request.model_xml = box_sdf; 
    spawn_srv.request.robot_namespace = "";


    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Successfully spawned box with size %.2f x %.2f x %.2f", size[0], size[1], size[2]);
    } else {
        ROS_ERROR("Failed to spawn the box.");
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "box_spawner"); 
	ros::NodeHandle nh;
    
	spawnObject(nh);
	return 0;
}
