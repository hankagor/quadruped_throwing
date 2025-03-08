#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <iostream>
#include <gazebo_msgs/SpawnModel.h>
#include <sstream>
#include <cstdlib>
// #include <../../../matplotlib-cpp/matplotlibcpp.h>
// #include "../../jumping/HRI.h"

using namespace std;

void reset(std::vector<double> start, std::vector<double> end, ros::Publisher pub, ros::Rate rate){
    std_msgs::Float64MultiArray msg;
    double cnt = 0, total_steps = 100, k = 0;
    while (k < 0.98) {
        k = std::min(cnt / total_steps, 1.0);
    
        std::vector<double> current_pos(3);
        current_pos[0] = k * start[0] + (1 - k) * end[0];
        current_pos[2] = k * start[2] + (1 - k) * end[2];
            
        msg.data = {0.02, current_pos[0], current_pos[1], current_pos[2], 1, 0.0};
        pub.publish(msg);
        ROS_INFO("Published: pos = [%f, %f, %f]", current_pos[0], current_pos[1], current_pos[2]);


        cnt += 1.0;
        rate.sleep();
    }
    ros::Duration(1.0).sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "catchobj_params_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/catchobj_params", 1);
    

    std::vector<double> start = {-0.35, 0.0, 0.15};
    std::vector<double> end = {-0.15, 0.0, 0.35};
    double goal = 0.5;
    if (argc > 1) {
        goal = atof(argv[1]);  // Get goal from command line
    }
    std::cout << goal << std::endl;
    std::vector<double> p = {-0.2, 0.0, goal}, r(3);
    for (int i = 0; i < 3; i++)
        r[i] = end[i] - start[i];    
    double p_xy = sqrt(p[1] * p[1] + p[2] * p[2]);

    // // physics-based controller
    double des_vel = p_xy * sqrt(9.81 / (end[0] - p[0] + p_xy));
    double vel = sqrt(2) / 2.0 * des_vel;

    /// quadratic
    //////
    // double total_time = 2* (end[0] - start[0]) / des_vel;
    // double sampling_time = 0.002;
    // double freq = 1.0 / sampling_time;
    // std::cout << total_time << "**" << des_vel << std::endl;
    // ros::Rate rate(freq); sqrt(2)
    // double a = des_vel / total_time;

    // double cnt = 0.0, total_steps = total_time * freq;
    // int close = 2;
    // std_msgs::Float64MultiArray msg;

    // // starting position
    // // msg.data = {sampling_time, start[0], start[1], start[2], close};
    // // pub.publish(msg);
    // // ROS_INFO("Starting position");
    // // ros::Duration(0.5).sleep();
    // ROS_INFO("Throwing...");
    // double k = 0.0;
    // while(k < 0.999){
    //     k = cnt / total_steps;
    //     if (k >= 0.96)
    //         close = 0;
    //     std::vector<double> current_pos(3);
    //     double current_time = cnt * sampling_time;
    //     current_pos[0] = start[0] + a * current_time * current_time / 2.0;
    //     current_pos[2] = start[2] + a * current_time * current_time / 2.0;
    //     double current_vel = k * des_vel;
    //     msg.data = {1.0, current_pos[0], current_pos[1], current_pos[2], static_cast<double>(close), current_vel, des_vel};
    //     pub.publish(msg);
    //     rate.sleep();
    //     cnt++;
    // }
    // ROS_INFO("Reset");
    // ros::Duration(0.1).sleep();
    // msg.data = {0.0, end[0], end[1], end[2], static_cast<double>(close), 0, 0};
    // pub.publish(msg);
    // // reset(start, end, pub, rate);
    // ros::Duration(2.5).sleep();
    

    /// linear
    ///////////
    double des_vel2 = des_vel;
    double total_time = (end[0] - start[0]) / des_vel2;
    double freq = 500.0;
    ros::Rate rate(freq); 
    double steps = total_time * freq;
    double time_step = 1.0 / freq;
    double cnt = 0.0;
    std::cout << total_time << std::endl;
    int close = 2;
    std_msgs::Float64MultiArray msg;
    // msg.data = {time_step, start[0], start[1], start[2], close};
    // pub.publish(msg);
    // ROS_INFO("reset");
    // ros::Duration(0.2).sleep();

    while (ros::ok()) {
        double k = std::min(cnt / steps, 1.0);
    
        if (k >= 0.98)
            close = 0;

        std::vector<double> current_pos(3);
        current_pos[0] = start[0] + des_vel2 * cnt * time_step;
        current_pos[2] = start[2] + des_vel2 * cnt * time_step;
            
        msg.data = {1.0, current_pos[0], current_pos[1], current_pos[2], static_cast<double>(close), des_vel};
        pub.publish(msg);
        ROS_INFO("Published: pos = [%f, %f, %f]", current_pos[0], current_pos[1], current_pos[2]);

        if (cnt * time_step >= total_time) {
            ROS_INFO("Reached target position.");
            break;
        }

        cnt += 1.0;
        rate.sleep();
    }

    ROS_INFO("Reset");
    ros::Duration(0.1).sleep();
    // reset(start, end, pub, rate);
    msg.data = {0.0, end[0], end[1], end[2], static_cast<double>(close), 0};
    pub.publish(msg);
    ros::Duration(2.5).sleep();

    return 0;
}
