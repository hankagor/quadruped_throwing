#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates

thrown = False
def model_state_callback(data):
    global thrown
    try:
        index = data.name.index('smaller_box')
        position = data.pose[index].position
        if position.x < 0.15:
            thrown = True
        if position.z < 0.15 and position.x > 0.15 and thrown:
            thrown = False
            rospy.loginfo(f"smaller_box Position: x={position.x}, y={position.y}, z={position.z}")
    except ValueError:
        k = 0
        # rospy.logwarn("Model 'smaller_box' not found in model states")

if __name__ == '__main__':
    thrown = False
    rospy.init_node('model_tracker')
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
    rospy.spin()
