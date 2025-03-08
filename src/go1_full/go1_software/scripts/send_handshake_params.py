#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy


class HandshakeParamSender:
    def __init__(self):
        self.params_pub = rospy.Publisher("/handshake_params", Float64MultiArray, queue_size=1)
        self.params = Float64MultiArray()

    
    def publish(self):
        """ Publishing convention:
                amplitude a_x
                amplitude a_z
                frequency omega_swing
                frequency omega_stance 
                kpCartesian 
                kdCartesian 
        """
        # random selection of different parameters 
        if numpy.random.random() > 0.5:
            self.params.data = [0.05, 0.05, 2, 2, 100, 2 ]
        else:
            self.params.data = [0.1, 0.1, 4, 4, 400, 8 ]
        self.params_pub.publish(self.params)


if __name__ == "__main__":
    rospy.init_node("param_pub_node", anonymous=True)

    paramSender = HandshakeParamSender()
    r = rospy.Rate(0.5) 
    while not rospy.is_shutdown():
        paramSender.publish()
        r.sleep()
