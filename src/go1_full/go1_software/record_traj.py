#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
import csv

# Define a callback to handle the pose and velocity data
def model_states_callback(data):
    try:
        # Assuming your object is the first model (index 0)
        # Change index based on your model's position in the list
        model_name = 'your_model_name'  # Replace with the actual model name
        index = data.name.index(model_name)
        pose = data.pose[index]
        velocity = data.twist[index]

        # Log position and velocity data
        print(f"Pose: {pose.position.x}, {pose.position.y}, {pose.position.z}")
        print(f"Velocity: {velocity.linear.x}, {velocity.linear.y}, {velocity.linear.z}")

        # Save to a CSV file
        with open('trajectory.csv', mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([pose.position.x, pose.position.y, pose.position.z,
                             velocity.linear.x, velocity.linear.y, velocity.linear.z])
    except ValueError:
        rospy.logerr("Model not found in /gazebo/model_states")

if __name__ == '__main__':
    rospy.init_node('trajectory_recorder', anonymous=True)

    # Subscribe to the /gazebo/model_states topic
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

    rospy.spin()  # Keep the script alive

