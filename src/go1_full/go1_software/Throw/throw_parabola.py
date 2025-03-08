#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "catchobj_params_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/catchobj_params", 1);
    double freq = 1000.0;
    ros::Rate rate(freq);  // 500 Hz

    std::vector<double> start = {-0.5, 0.0, 0.4};  // Starting position
    std::vector<double> end = {0.1, 0.0, 1.0};     // Ending position

    double total_time = 0.1;
    double steps = total_time * freq;
    double time_step = 1.0 / freq;
    double cnt = 0.0;
    
    int close = 1;
    std_msgs::Float64MultiArray msg;
    msg.data = {time_step, start[0], start[1], start[2], 1};
    pub.publish(msg);
    ROS_INFO("reset");
    ros::Duration(0.2).sleep();

    double start_time = ros::Time::now().toSec();
    
    // Parabola coefficient for z-axis (negative for falling motion)
    double a = -4.0;  // Negative coefficient to simulate a downward trajectory

    while (ros::ok()) {
        double k = std::min(cnt / steps, 1.0);
        ROS_INFO("Step = %f, Percent = %f", cnt, k);

        if (k >= 0.5 && close) {
            double end_time = ros::Time::now().toSec();
            ROS_INFO("elapsed time = %f", end_time - start_time);
            close = 0;
        }

        // Parabolic equation for the z-axis
        double current_z = (a * k * k) + ((1.0 - k) * start[2]) + (k * end[2]);

        // Linear interpolation for x and y
        std::vector<double> current_pos(3);
        current_pos[0] = (1.0 - k) * start[0] + k * end[0];  // Linear for x
        current_pos[1] = (1.0 - k) * start[1] + k * end[1];  // Linear for y
        current_pos[2] = current_z;                          // Parabolic for z

        msg.data = {time_step, current_pos[0], current_pos[1], current_pos[2], static_cast<double>(close)};
        pub.publish(msg);
        ROS_INFO("Published: pos = [%f, %f, %f]", current_pos[0], current_pos[1], current_pos[2]);

        if (cnt * time_step >= total_time) {
            ROS_INFO("Reached target position.");
            break;
        }

        cnt += 1.0;
        rate.sleep();
    }

    double end_time = ros::Time::now().toSec();
    ROS_INFO("elapsed time = %f", end_time - start_time);
    msg.data = {0.5, start[0], start[1], start[2], 1};
    pub.publish(msg);
    ros::Duration(1.0).sleep();
    ROS_INFO("reset");

    return 0;
}

