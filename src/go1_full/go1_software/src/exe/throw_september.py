import numpy as np
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float32, Float64MultiArray, String

import threading
# publishing ? 
class PublishThread(threading.Thread):
    def __init__(self, params_pub):
        # super.__init__(daemon=True)
        threading.Thread.__init__(self)
        self.params_pub = params_pub
        self.stop_event = threading.Event()

    def run(self):
        rate = rospy.Rate(100)
        while not self.stop_event.is_set():
            T_remain1 = T_remain - (time.time() - t_remain_time)
            params.data = [T_remain1, x_catch_robot, y_catch_robot, z_catch_robot] # updates the position
            print('++++++++++++++',T_remain1, x_catch_robot, y_catch_robot, z_catch_robot)
            self.params_pub.publish(params)
            rate.sleep()  # Adjust the sleep time to control the publishing rate

    def stop(self):
        self.stop_event.set()


# initilization

def init_ros():

    rospy.init_node('publisher_node')
    rate = rospy.Rate(10)
    print("Publisher Node Started")
    params_pub = rospy.Publisher("/obj_params", Float64MultiArray, queue_size=1)
    params = Float64MultiArray()

    return params_pub, params

# main function
def throw():
    params_pub, params = init_ros()

    params.data = [10, 0, 0, 0.23]
    params_pub.publish(params)

    publish_thread = PublishThread(params_pub)


if __name__=='__main__':
    try:
        throw()

    except rospy.ROSInterruptException:
        pass

