#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64.h>

class BoxPosePublisher {
public:
    BoxPosePublisher(double frequency) : rate(frequency) {
        pub_x = nh.advertise<std_msgs::Float64>("box_x", 10);
        pub_y = nh.advertise<std_msgs::Float64>("box_y", 10);
        pub_z = nh.advertise<std_msgs::Float64>("box_z", 10);

        sub = nh.subscribe("/gazebo/model_states", 10, &BoxPosePublisher::callback, this);

        model_name = "smaller_box";
        object_found = false;

        ROS_INFO("Tracking poses for model: %s at %.2f Hz", model_name.c_str(), frequency);
    }

    void spin() {
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep(); 
        }
    }

private:
    void callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        try {
            auto it = std::find(msg->name.begin(), msg->name.end(), model_name);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);

                double x = msg->pose[index].position.x;
                double y = msg->pose[index].position.y;
                double z = msg->pose[index].position.z;

                std_msgs::Float64 msg_x, msg_y, msg_z;
                msg_x.data = x;
                msg_y.data = y;
                msg_z.data = z;

                pub_x.publish(msg_x);
                pub_y.publish(msg_y);
                pub_z.publish(msg_z);

                ROS_INFO("Published: x=%.2f, y=%.2f, z=%.2f", x, y, z);
                object_found = true;
            } 
            else {
                if (object_found) {
                    ROS_WARN("Model %s deleted", model_name.c_str());
                    object_found = false;
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error in callback: %s", e.what());
        }
    }

    ros::NodeHandle nh;
    ros::Publisher pub_x, pub_y, pub_z;
    ros::Subscriber sub;
    ros::Rate rate; 
    std::string model_name;
    bool object_found;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "box_pose_publisher");

    double frequency = 100.0; // Desired frequency in Hz
    BoxPosePublisher boxPosePublisher(frequency);

    boxPosePublisher.spin();

    return 0;
}
