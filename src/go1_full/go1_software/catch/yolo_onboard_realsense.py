#!/usr/bin/env python3
import cv2
from PIL import Image, ImageFont
import numpy as np
import math
# yolo_onboard_realsense.py
from go1_full.go1_software.catch.utils_yolo_onboard_realsense import *
from decimal import Decimal, ROUND_HALF_UP
# import pyrealsense2 as rs  # Commented out hardware
from datetime import datetime, timedelta
import time

from ultralytics import YOLO

import rospy
from std_msgs.msg import Float32, Float64MultiArray, String
from std_msgs.msg import ColorRGBA
# from quadruped.msg import Coords

# Define the colors to detect
YELLOW = [0, 255, 255]  # Yellow in BGR colorspace
GREEN = [80, 250, 0]  # Green in BGR colorspace
RED = [0, 0, 255]  # Red in BGR colorspace
TIME_TO_CATCH = 0.04 # 0.1 #0.13
Z_EPS = 0.26 #0.2 #0.15
Y_EPS = 0.26 #0.2 #0.15

import threading

class PublishThread(threading.Thread):
    def __init__(self, params_pub):
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

class ColorParamSender:
    def __init__(self):
        self.colors_pub = rospy.Publisher("/go1_led_color", ColorRGBA, queue_size=1)
        self.color = ColorRGBA()

    def publish(self,off=False):
        """ Publishing convention:
                r, g, b, a (a can be set to 1, not being used by Go1 anyways)
        """
        if off:
            self.color.r = 0 #255
            self.color.g = 0
            self.color.b = 0
            self.color.a = 1 
        else:
            self.color.r = 0 #255
            self.color.g = 255
            self.color.b = 0
            self.color.a = 1 
        self.colors_pub.publish(self.color)
        print('publishing color:', self.color)

        # rospy.spin()


def catch():
    global params, T_remain, x_catch_robot, y_catch_robot, z_catch_robot, t_remain_time
    # parameters initilization
    frame_number = 0  
    start_experiment = 0
    not_done = 1
    exp_once = 1
    first_z = 1
    get_obj = 1
    get_obj2=0
    first_get_obj = 0
    closed = False
    send = True
    send_start = True
    T_remain_updated = 0
    Ztarget=1000
    j = 0
    X = []
    Y = []
    Z = []
    frame_numbers = []
    T=[]
    catch_first = True
    once = True

    x_feet = 0          # Set Exact start position of feet
    y_feet = 0 - 0.25 - 0.1 #-0.20      # Set Exact start position of feet
    z_feet = 0.23+0.05+0.05 +0.05+0.05     # Set Exact start position of feet

    T_remain, x_catch_robot, y_catch_robot, z_catch_robot = 1000000, 0,0,0.23
    t_remain_time = 1000000000000000000000
    close_time = time.time()

    # Initialize ROS Node and Topic
    params_pub, params, params_pub2, params2 = init_ros()

    params.data = [10, 0, 0, 0.23]
    params_pub.publish(params)

    publish_thread = PublishThread(params_pub)

    
    colorSender = ColorParamSender()
    colorSender.publish(off=True)

    # Initialize Camera (Commented out hardware initialization)
    # pipe, intr = init_camera()

    # model = YOLO("YOLO/best_onboard.pt")
    model = YOLO("/home/hanka/catkin_ws_go1_full/src/go1_full/go1_software/catch/YOLO/best_yolo_1212_2023.pt")
    # model = YOLO("YOLO/best_yolo_2202_2024.pt")

    # Initialize Video output (commented out)
    out = init_video()
    out_full_annotated = init_video_full_annotated()
    out_full = init_video_full()

    # Initialize Text file
    t=str(time.time())
    text_file_cart = open('data/'+t + '_object_coordinates.txt', 'w')

    params2.data = t

    # Sleep for 6 seconds
    time.sleep(3)

    colorSender.publish(off=True)

    start_throw = time.time()
    throw_delta = 8
    print("Get Ready...")

    while (time.time()-start_throw) < throw_delta:
        # Get object Coordinates (Commented out camera-related logic)
        # found,Xtarget, Ytarget, Ztarget, annotated_image, color_image, frame_image  = get_object_coordinates(pipe,intr,model)
        # out_full_annotated.write(annotated_image)
        # out_full.write(color_image)

        found = True  # Dummy value for testing without hardware
        Xtarget, Ytarget, Ztarget = 1, 1, 1  # Dummy values
        
        if get_obj:
            if get_obj2:
                first_get_obj = 1
                get_obj = 0
            get_obj2 = 1

            print("Throw!!")
            colorSender.publish(off=False)

        if found and first_get_obj and Ztarget:
            if first_z:
                first_z = 0 
                Z_old = Ztarget
                Y_old = Ytarget

            if abs(Ztarget-Z_old)>Z_EPS and abs(Ytarget-Y_old)>Y_EPS and exp_once:
                print("Z_target",Ztarget)
                print("Z_old",Z_old)
                print("Y_target",Ytarget)
                print("Y_old",Y_old)
                                
                start_experiment = 1
                exp_once=0
                start_time = time.time()

                params_pub2.publish(params2)

            if start_experiment:
                X.append(Xtarget)
                Y.append(Ytarget)
                Z.append(Ztarget)
                frame_numbers.append(frame_number)
                T.append(time.time()-start_time)
                if once:
                    once = False
                    publish_thread.start()

                if j>1:
                    t_remain_time = time.time()
                    T_all, x_pred, y_pred, z_pred = fit(frame_numbers,X,Y,Z,T)

                    x_catch_cam, y_catch_cam, z_catch_cam, T_remain = reach_robot_new(x_pred, y_pred, z_pred, start_time, T_all,x_feet, y_feet, z_feet)

                    x_catch_robot, y_catch_robot, z_catch_robot = camref2robotref(x_catch_cam, y_catch_cam, z_catch_cam)

                    text_file_cart.write(f"Frame: {frame_number}, x: {Xtarget}, y: {Ytarget}, z: {Ztarget}\n")
                    text_file_cart.write(f"time, x_catch, y_catch, z_catch ,T_remain:  ,{time.time()}, {x_catch_robot} ,{y_catch_robot} ,{z_catch_robot}, {T_remain},\n")
                    text_file_cart.write(f'x_pred: {x_pred}\n')
                    text_file_cart.write(f'y_pred: {y_pred}\n')
                    text_file_cart.write(f'z_pred: {z_pred}\n')
                    
                    print("time, x_object, y_object, z_object: " ,time.time()-start_throw, Xtarget ,Ytarget ,Ztarget)
                    print("x_catch, y_catch, z_catch ,T_remain: " , x_catch_robot, y_catch_robot, z_catch_robot , T_remain)

                j = j + 1

        if closed:
            T_remain = 0
            T_remain_updated = 0

        print("T_remain ----------------->",T_remain)
        print('time---------------------->',time.time())

        if T_remain<0.2:
            closed = True

        frame_number += 1

    T_remain, x_catch_robot, y_catch_robot, z_catch_robot = 10000, 0,0,0.23
    params.data = [10, 0, 0, 0.23]
    params_pub.publish(params)

    publish_thread.stop()
    publish_thread.join()

if __name__=='__main__':
    try:
        catch()
    except rospy.ROSInterruptException:
        pass

