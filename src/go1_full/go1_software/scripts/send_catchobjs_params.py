#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np


class CatchObsSender:
    def __init__(self):
        self.params_pub = rospy.Publisher("/catchobj_params", Float64MultiArray, queue_size=1)
        self.params = Float64MultiArray()

    
    def publish(self):
        """ Publishing convention:
            time_to_close (s)
            x pos in robot frame 
            y pos in robot frame
            z pos in robot frame 
        """
        # random selection of different parameters 
        if np.random.random() > 0.5:
            self.params.data = [0.1, 0.1, -0.05, 0.17]
            # self.params.data = [0.1, 
            #                     0.2*(2*np.random.random()-1), 
            #                     0.1*(2*np.random.random()-1), 
            #                     0.2 + 0.1*(2*np.random.random()-1)]
            # conservative hardware limits 
            self.params.data = [0.1, 
                                0.1*(2*np.random.random()-1), 
                                0.05*(2*np.random.random()-1), 
                                0.2 + 0.05*(2*np.random.random()-1)]
        else:
            self.params.data = [0.1, 0, 0, 0.23 ]
        # self.params.data = [0.1, 0, 0, 0.23 ]
        print(self.params.data)
        self.params_pub.publish(self.params)


if __name__ == "__main__":
    rospy.init_node("param_pub_node", anonymous=True)

    paramSender = CatchObsSender()
    r = rospy.Rate(0.5) 
    while not rospy.is_shutdown():
        paramSender.publish()
        r.sleep()
