#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import cv2
from PIL import Image, ImageFont
import math
import time
import pyrealsense2 as rs
import torch
from scipy.stats import multivariate_normal


import cv2.aruco as aruco


import rospy
from std_msgs.msg import Float32, Float64MultiArray, String
# from quadruped.msg import Coords


# INITIALIZATION

def init_ros():
    # pub = rospy.Publisher('talking_topic', Coords, queue_size=10)
    rospy.init_node('publisher_node')
    rate = rospy.Rate(10)
    print("Publisher Node Started")
    params_pub = rospy.Publisher("/catchobj_params", Float64MultiArray, queue_size=1)
    params = Float64MultiArray()

    params_pub2 = rospy.Publisher("/start_recording_params", String, queue_size=1)
    params2 = String()

    return params_pub, params, params_pub2, params2

def init_camera():
    pipe = rs.pipeline()
    cfg  = rs.config()

    cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 30)

    profile = pipe.start(cfg)

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    return pipe, intr

def init_video(FRAME_RATE=30, FRAME_WIDTH=640, FRAME_HEIGHT=480):
    # Create a VideoWriter object with the obtained values
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can choose other codecs like 'MJPG' or 'X264'
    out = cv2.VideoWriter('data/'+str(time.time()) + '_robot_catch.avi', fourcc, FRAME_RATE, (FRAME_WIDTH, FRAME_HEIGHT))

    return out

def init_video_full_annotated(FRAME_RATE=30, FRAME_WIDTH=640, FRAME_HEIGHT=480):
    # Create a VideoWriter object with the obtained values
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can choose other codecs like 'MJPG' or 'X264'
    out = cv2.VideoWriter('data/'+str(time.time()) + '_robot_catch_full_annotated.avi', fourcc, FRAME_RATE, (FRAME_WIDTH, FRAME_HEIGHT))

    return out

def init_video_full(FRAME_RATE=30, FRAME_WIDTH=640, FRAME_HEIGHT=480):
    # Create a VideoWriter object with the obtained values
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can choose other codecs like 'MJPG' or 'X264'
    out = cv2.VideoWriter('data/'+str(time.time()) + '_robot_catch_full.avi', fourcc, FRAME_RATE, (FRAME_WIDTH, FRAME_HEIGHT))

    return out

def init_textfile(t):
    # Open a text file for writing frame numbers and middle coordinates
    text_file_cart = open('data/'+t + '_object_coordinates.txt', 'w')
    return text_file_cart


# ROBOT / OBJECT

def get_object_coordinates_width(pipe,intr,model,T_remain,Z_old,draw = True, theta=np.pi*(10/180)):
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    if not depth_frame or not color_frame:
        return 0, 0, 0, 0, 0, 0, 0

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                        alpha = 0.5), cv2.COLORMAP_JET)

    # image = Image.fromarray(color_image)

    # t1=time.time()
    # Run YOLOv8 tracking on the frame, persisting tracks between frames
    results = model.track(color_image, stream=False,show=False, persist=True) 
    # results = model.predict(color_image)#, persist=True) 
    # print('2',time.time()-t1)

    conf = results[0].boxes.conf
    xyxy = results[0].boxes.xyxy

    annotated_frame = color_image

    # t1 = time.time()
    if draw:
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
    # print('time to annotate', time.time()-t1)

    

    # Check if conf is not empty
    if conf.numel() > 0:
        if conf.numel() == 1:
            # Find max and argmax of conf

            max_conf = conf[0].tolist()
            X = xyxy[0].tolist()
            x1,y1,x2,y2 = X[0],X[1],X[2],X[3]

            
            # print(x1,y1,x2,y2)

        else:
            # Find max and argmax of conf
            max_conf = torch.max(conf).item()
            argmax_conf = torch.argmax(conf).item()

            X = xyxy[argmax_conf].tolist()
            x1,y1,x2,y2 = X[0],X[1],X[2],X[3]

            
            # print(x1,y1,x2,y2)

        # Set a threshold
        threshold = 0.65
        # Compare max_conf to the threshold
        print("conf", conf)
        print("max_conf",max_conf)
        if max_conf > threshold:
            middle_x = (x1+x2)/2
            middle_y = (y1+y2)/2

            center = (middle_x, middle_y)  # Get the (x, y) coordinates
            radius = 10  # Radius of the circle
            color = (255, 0, 0)  # Color in BGR format (red in this case)
            thickness = 2  # Thickness of the circle's outline

            # if draw:
            #     frame = cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 5) 

            dist = depth_frame.get_distance(int(middle_x), int(middle_y))*1000 #convert to mm
            print(dist)
            
            if dist>0 or T_remain>0.5:
                Ztemp = dist
                print("Depth",Ztemp)
                width_pixel = abs(x1-x2)
                image_width = 640
                sensor_width = 0.41 # 0.38
                width_perceived = (width_pixel/image_width) *sensor_width
                width_real = 0.095
                print("Depth estimation",intr.fx*(width_real/width_perceived))

            else:
                # MAYBE ADD CONDITION TO ONLY USE THIS WHEN VERY CLOSE TO CAM
                width_pixel = abs(x1-x2)
                image_width = 640
                sensor_width = 0.41 # 0.375 #0.38
                width_perceived = (width_pixel/image_width) *sensor_width
                width_real = 0.095
                Ztemp = intr.fx*(width_real/width_perceived)
                # if (Ztemp>Z_old+300) or (Ztemp<Z_old-300):
                #     Ztemp = 0
                print("Depth estimation",intr.fx*(width_real/width_perceived))

            #calculate real world coordinates
            Xtemp = Ztemp*(middle_x -intr.ppx)/intr.fx
            Ytemp = Ztemp*(middle_y -intr.ppy)/intr.fy

            Xtarget = Xtemp #- 35 #35 is RGB camera module offset from the center of the realsense
            Ytarget = -(Ztemp*math.sin(theta) + Ytemp*math.cos(theta))
            Ztarget = Ztemp*math.cos(theta) - Ytemp*math.sin(theta)
            

            return 1, Xtarget/1000, Ytarget/1000, Ztarget/1000, annotated_frame, color_image, frame
        else:
            return 0, 0, 0, 0, annotated_frame, color_image, 0
    else:
        return 0, 0, 0, 0, annotated_frame, color_image, 0

def get_object_coordinates(pipe,intr,model,draw = True, theta=np.pi*(10/180)):
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    if not depth_frame or not color_frame:
        return 0, 0, 0, 0, 0, 0, 0

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                        alpha = 0.5), cv2.COLORMAP_JET)
    
    # image = Image.fromarray(color_image)

    # t1=time.time()
    # Run YOLOv8 tracking on the frame, persisting tracks between frames
    results = model.track(color_image, stream=False,show=False, persist=True) 
    # results = model.predict(color_image)#, persist=True) 
    # print('2',time.time()-t1)

    conf = results[0].boxes.conf
    xyxy = results[0].boxes.xyxy

    annotated_frame = color_image

    # t1 = time.time()
    if draw:
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
    # print('time to annotate', time.time()-t1)

 

    # Check if conf is not empty
    if conf.numel() > 0:
        if conf.numel() == 1:
            # Find max and argmax of conf

            max_conf = conf[0].tolist()
            X = xyxy[0].tolist()
            x1,y1,x2,y2 = X[0],X[1],X[2],X[3]

            
            # print(x1,y1,x2,y2)

        else:
            # Find max and argmax of conf
            max_conf = torch.max(conf).item()
            argmax_conf = torch.argmax(conf).item()

            X = xyxy[argmax_conf].tolist()
            x1,y1,x2,y2 = X[0],X[1],X[2],X[3]

            
            # print(x1,y1,x2,y2)

        # Set a threshold
        threshold = 0.7
        # Compare max_conf to the threshold
        if max_conf > threshold:
            middle_x = (x1+x2)/2
            middle_y = (y1+y2)/2

            center = (middle_x, middle_y)  # Get the (x, y) coordinates
            radius = 10  # Radius of the circle
            color = (255, 0, 0)  # Color in BGR format (red in this case)
            thickness = 2  # Thickness of the circle's outline

            # if draw:
            #     frame = cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 5) 



            dist = depth_frame.get_distance(int(middle_x), int(middle_y))*1000 #convert to mm
            # print(dist)
            surrounding_values = depth_image[int(middle_y-7):int(middle_y+8),int(middle_x-7):int(middle_x+8)]

            if not dist:
                non_zero_values = surrounding_values[surrounding_values!=0]
                dist = np.mean(non_zero_values)
                if np.isnan(dist):
                    dist = 0

            print(surrounding_values)
            # print(dist)
            # print(depth_image[int(middle_y), int(middle_x)])

            #calculate real world coordinates
            Xtemp = dist*(middle_x -intr.ppx)/intr.fx
            Ytemp = dist*(middle_y -intr.ppy)/intr.fy
            Ztemp = dist

            Xtarget = Xtemp - 35 #35 is RGB camera module offset from the center of the realsense
            Ytarget = -(Ztemp*math.sin(theta) + Ytemp*math.cos(theta))
            Ztarget = Ztemp*math.cos(theta) - Ytemp*math.sin(theta)
            return 1, Xtarget/1000, Ytarget/1000, Ztarget/1000, annotated_frame, color_image, frame
        else:
            return 0, 0, 0, 0, annotated_frame, color_image, 0
    else:
        return 0, 0, 0, 0, annotated_frame, color_image, 0

def fit(frames,x_gt,y_gt,z_gt,T):
    g = 9.8
    frame_rate = 30
    time_step = 1/frame_rate

    i = 0
    t = 0
    frame_old = frames[0]

    y_pred = []
    x_pred = []
    z_pred = []


    A = np.zeros((3,3))
    b = np.zeros((3,1))

    T_all = np.linspace(0,4,num=200)  # DEFINE A LONGER T_ALL

    t = 0

    # print('T_real',T)
    # print('T_all',T_all)

    

    j = 0

    # T = []
    # T.append(0)

    # for frame in frames:
    #     if j>0:
    #         d_frame = frame - frame_old
    #         t = t + d_frame*time_step
    #         T.append(t)
    #     j = j + 1
    #     frame_old = frame

    lambda_ = 1 # to account for gravity

    x_pred = []
    y_pred = []
    z_pred = []
    #Perform polynoial regression x = at + b
    T_arr = np.array(T)

    T_poly = build_poly(T_arr, degree=1)
    w_x = least_squares(x_gt, T_poly)

    T_all_poly = build_poly(T_all, degree=1)
    x_pred = T_all_poly@w_x

    #Perform polynoial regression z = at + b
    T_poly = build_poly(T_arr, degree=1)
    w_z = least_squares(z_gt, T_poly)

    T_all_poly = build_poly(T_all, degree=1)
    z_pred = T_all_poly@w_z
    
    #Perform polynoial regression y = at^2 + bt + c

    A[0,0] = len(T_arr)
    A[0,1] = sum(T_arr)
    A[1,0] = A[0,1]
    A[0,2] = sum(T_arr**2)
    A[1,1] = A[0,2]
    A[2,0] = A[0,2]
    A[1,2] = sum(T_arr**3)
    A[2,1] = A[1,2]
    A[2,2] = sum(T_arr**4) + lambda_

    b[0] = sum(y_gt)
    b[1] = np.dot(T_arr,y_gt)
    b[2] = np.dot(T_arr**2,y_gt) - 0.5*lambda_*g

    x = np.linalg.solve(A,b)

    t_eval = np.ones((3,len(T_all)))
    t_eval[2,:] = T_all**2
    t_eval[1,:] = T_all

    y_pred = x.T @ t_eval

    return T_all, x_pred, y_pred[0], z_pred

def cart2pix(X_target, Y_target, Z_target,intr):
    dist = Z_target
    x_pix = ((X_target+35)/dist)*intr.fx + intr.ppx
    y_pix = ((-Y_target)/dist)*intr.fy + intr.ppy

    return x_pix, y_pix

def estimate_robotpos(pipe,intr,color):
    print("Start estimating Robot position")

    # Find position of robot
    robot_xs = []
    robot_ys = []
    robot_zs = []

    start_estimate = time.time()
    estimate_delta = 3
    while (time.time()-start_estimate) < estimate_delta:
        # found,Xtarget, Ytarget, Ztarget,color_image, frame_image  = get_object_coordinates(pipe,intr,color)
        found,Xtarget, Ytarget, Ztarget,color_image, frame_image  = find_aruco(pipe,intr)
        if found and Ztarget:
            robot_xs.append(Xtarget)
            robot_ys.append(Ytarget)
            robot_zs.append(Ztarget)
    x_robot = np.mean(robot_xs)
    y_robot = np.mean(robot_ys)
    z_robot = np.mean(robot_zs)

    print("Done estimating Robot position")
    print('x_robot:', x_robot,'y_robot', y_robot,'z_robot', z_robot)

    return x_robot, y_robot, z_robot

def reach_robot(x_pred, y_pred, z_pred,start_time, T_all):
    # Find x,y,z, T when the object will reach x_robot
    indices = [index for index, value in enumerate(z_pred) if value < (0.35)] #0.35
    if indices != []:
        first_index = indices[0]
    else:
        first_index = -1
    T_thresh = T_all[first_index]

    # Object coordinates when reach robot (Camera reference frame)
    x_catch_cam = x_pred[first_index]
    y_catch_cam = y_pred[first_index]
    z_catch_cam = z_pred[first_index]

    # print("T_all",T_all)
    # print("x_pred",x_pred)
    # print("y_pred",y_pred)
    # print("z_pred",z_pred)
    # print("---------------")
    # print("x_catch_cam y_catch_cam z_catch_cam",x_catch_cam, y_catch_cam, z_catch_cam,first_index)

    # Time remaining to reach x_thresh
    T_remain = T_thresh - (time.time()-start_time)

    return x_catch_cam, y_catch_cam, z_catch_cam, T_remain


def reach_robot_new(x_pred, y_pred, z_pred,start_time, T_all,x_feet, y_feet, z_feet):
    # Find x,y,z closest to robot feet
    dist = (x_pred-x_feet)**2 + (y_pred-y_feet)**2 + (z_pred-z_feet)**2
    index = np.argmin(dist)

    T_thresh = T_all[index]

    # Object coordinates when reach robot (Camera reference frame)
    x_catch_cam = x_pred[index]
    y_catch_cam = y_pred[index]
    z_catch_cam = z_pred[index]

    # Time remaining to reach x_thresh
    T_remain = T_thresh - (time.time()-start_time)

    if z_catch_cam<0.30:
        z_catch_cam = 0.30

    return x_catch_cam, y_catch_cam, z_catch_cam, T_remain


def reach_robot_GMM(x_pred, y_pred, z_pred,start_time, T_all):
    # Find x,y,z, T when the object will reach x_robot


    # GMM_params = {"n_components": 1, "means": [[0.00671891, -0.014078208571428571, -0.2761909387755102]], "covariances": [[[0.00364684700055623, 0.0012510385555237792, 0.0002380099303773469], [0.0012510385555237792, 0.01631334242313883, -0.0024584186763106997], [0.0002380099303773469, -0.0024584186763106997, 0.0013018086595676802]]], "weights": [1.0], "bic_scores": [56.77080413901909, 33.52148366356116, 8.083857633087787, -20.63576181995786]}
    # GMM_params = {"n_components": 1, "means": [[0.00671891, -0.014078208571428571, 0.2761909387755102]], "covariances": [[[0.01631334242313883, 0.0012510385555237792, -0.0024584186763106997], [0.0012510385555237792, 0.00364684700055623, 0.0002380099303773469], [-0.0024584186763106997, 0.0002380099303773469, 0.0013018086595676802]]], "weights": [1.0], "bic_scores": [56.77080413901909, 33.52148366356116, 8.083857633087787, -20.63576181995786]}
    # New Fitting
    GMM_params = {"n_components": 1, "means": [[0.00173176, 0, 0.28435418]], "covariances": [[[1.84258822e-02, -1.43069977e-19, -2.07667791e-03], [-1.43069977e-19, 6.25605247e-03, 0], [-2.07667791e-03, 0, 1.72678182e-03]]], "weights": [1.0], "bic_scores": [56.77080413901909, 33.52148366356116, 8.083857633087787, -20.63576181995786]}

    n_components, means, covariances, weights =  GMM_params['n_components'],GMM_params['means'],GMM_params['covariances'],GMM_params['weights']

    means[0][0],means[0][1], means[0][2] = -means[0][1], means[0][0]-0.25, means[0][2] + 0.05
    # print(means)

    # mean = np.array([0,-0.25,0.27])
    # covariance = np.array([[0.01631334242313883, 0.0012510385555237792, -0.0024584186763106997], [0.0012510385555237792, 0.00364684700055623, 0.0002380099303773469], [-0.0024584186763106997, 0.0002380099303773469, 0.0013018086595676802]])

    max_ = 0

    x_pred, y_pred, z_pred = np.array(x_pred), np.array(y_pred), np.array(z_pred)
    traj = np.column_stack((x_pred, y_pred, z_pred))

    index = 0
    for j, x_tr in enumerate(traj):
        f=0
        for i in range(n_components):
            # print(type(x_tr), type(means[i]), type(covariances[i]))
            pdf = weights[i]* multivariate_normal.pdf(x_tr, mean=means[i], cov=covariances[i])
            # pdf = weights[i]* multivariate_normal.pdf(x_tr, mean=mean, cov=covariance)
            f = f + pdf
        # print(x_tr,f)
        if f>max_:
            max_ = f
            x_max = x_tr
            index = j



    T_thresh = T_all[index]

    # Object coordinates when reach robot (Camera reference frame)
    x_catch_cam = x_pred[index]
    y_catch_cam = y_pred[index]
    z_catch_cam = z_pred[index]

    # Time remaining to reach x_thresh
    T_remain = T_thresh - (time.time()-start_time)

    return x_catch_cam, y_catch_cam, z_catch_cam, T_remain




def camref2robotref(x_catch_cam, y_catch_cam, z_catch_cam,head_center_robot = 0.25,z_offset = 0.1):
    z_catch_robot = z_catch_cam - z_offset
    y_catch_robot = - x_catch_cam 
    x_catch_robot =  y_catch_cam + head_center_robot

    return x_catch_robot, y_catch_robot, z_catch_robot

# z front
# x up
# y left


def robotref2camref(x_catch_robot, y_catch_robot, z_catch_robot, head_center_robot = 0.25, z_offset = 0.1):
    x_catch_cam = -y_catch_robot
    z_catch_cam = z_catch_robot + z_offset
    y_catch_cam = x_catch_robot - head_center_robot

    return x_catch_cam, y_catch_cam, z_catch_cam


def find_aruco(pipe,intr):
    # Load the ArUco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters()
    marker_size = 150 #70  # Marker size in millimeters
    cameraMatrix = np.array([[intr.fx, 0, intr.ppx],
                            [0, intr.fy, intr.ppy],
                            [0, 0, 1]])
    


    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    if not depth_frame or not color_frame:
        return 0, 0, 0, 0, 0, 0

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                        alpha = 0.5), cv2.COLORMAP_JET)
    image = Image.fromarray(color_image)
    result = np.asarray(image)

    # Convert the image to grayscale
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        for i in range(len(ids)):
            if ids[i] == 0:  # Replace 0 with your marker's ID
                # Calculate the center (X, Y) of the detected marker
                middle_x = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                middle_y = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)


                # Draw a rectangle around the detected marker
                aruco.drawDetectedMarkers(color_image, corners)

                # # Calculate the marker's pose
                # rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs)

                # # Draw axis and orientation of the marker
                # aruco.drawAxis(image, cameraMatrix, distCoeffs, rvecs, tvecs, 2 * marker_size)

                # # Print the pose of the marker
                # print(f"Marker 0 found at (X, Y, Z): {tvecs[0][0]}mm, {tvecs[0][1]}mm, {tvecs[0][2]}mm")

                dist = depth_frame.get_distance(int(middle_x), int(middle_y))*1000 #convert to mm

                #calculate real world coordinates
                Xtemp = dist*(middle_x -intr.ppx)/intr.fx
                Ytemp = dist*(middle_y -intr.ppy)/intr.fy
                Ztemp = dist

                Xtarget = Xtemp - 35 #35 is RGB camera module offset from the center of the realsense
                Ytarget = -(Ztemp*math.sin(theta) + Ytemp*math.cos(theta))
                Ztarget = Ztemp*math.cos(theta) - Ytemp*math.sin(theta)

                return 1, Xtarget/1000, Ytarget/1000, Ztarget/1000, color_image, frame
    return 0, 0, 0, 0, color_image, 0



# VISION / VISUALIZATION

def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 20, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 20, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 20, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 20, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

def pred_on_video(x_pred, y_pred, z_pred, intr, color_image):
    # Visualize prediction on video
    x_pix = []
    y_pix = []
    for i in range(len(x_pred)):
        x_pix1, y_pix1 = cart2pix(x_pred[i], y_pred[i], z_pred[i],intr)
        x_pix.append(x_pix1)
        y_pix.append(y_pix1)

    for i in range(len(x_pix)):
        center = (int(x_pix[i]), int(y_pix[i]))  # Get the (x, y) coordinates
        radius = 10  # Radius of the circle
        color = (0, 0, 255)  # Color in BGR format (red in this case)
        thickness = 2  # Thickness of the circle's outline
        # Draw a circle at (x, y)
        cv2.circle(color_image, center, radius, color, thickness)


# REGRESSION

def build_poly(x, degree):
    """polynomial basis functions for input data x, for j=0 up to j=degree.

    Args:
        x: numpy array of shape (N,), N is the number of samples.
        degree: integer.

    Returns:
        poly: numpy array of shape (N,d+1)
    """
    poly = np.ones((x.size,degree+1))
    for i in range(degree+1):
        poly[:,i] = (x.T)**i
    return poly

def least_squares(y, tx):
    """Calculate the least squares solution.
       returns mse, and optimal weights.

    Args:
        y: numpy array of shape (N,), N is the number of samples.
        tx: numpy array of shape (N,D), D is the number of features.

    Returns:
        w: optimal weights, numpy array of shape(D,), D is the number of features.
        mse: scalar.
    """
    tx = np.array(tx)
    w = np.linalg.solve(tx.T @ tx,tx.T @ np.array(y))
    return w


# def write_depth_frame_to_file(depth_frame, file):
#     """
#     Writes the depth frame to a text file.

#     Parameters:
#     - depth_frame: The depth frame to be written.
#     - file: The file object to write to.
#     """
#     for y in range(depth_frame.height):
#         for x in range(depth_frame.width):
#             depth_value = depth_frame.get_distance(x, y)
#             file.write(f"{depth_value:.6f} ")
#         file.write('\n')

def init_depth_textfile():
    # Open a text file for writing frame numbers and middle coordinates
    text_file_cart = open(str(time.time()) + 'depth.txt', 'w')
    return text_file_cart

def write_depth_frame_to_file(depth_frame, file, frame_number):
    """
    Appends the depth frame and frame number to a text file.

    Parameters:
    - depth_frame: The depth frame to be appended.
    - file: The file object to write to.
    - frame_number: The frame number to be written.
    """
    depth_values = depth_frame.get_data()
    
    # Write frame number
    file.write(f"Frame: {frame_number}\n")

    # Write depth values
    np.savetxt(file, depth_values, fmt='%.6f', newline='\n')
    file.write('\n')  # Add an extra newline after each depth frame

import re

def read_depth_frames_from_file(filename):
    """
    Reads frame numbers and depth values from a text file.

    Parameters:
    - filename: The name of the text file.

    Returns:
    - frames: A list of dictionaries, where each dictionary contains
              the frame number and depth values for a frame.
    """
    frames = []
    current_frame = None

    with open(filename, 'r') as file:
        for line in file:
            if line.startswith('Frame:'):
                if current_frame is not None:
                    frames.append(current_frame)
                frame_number = int(re.search(r'\d+', line).group())
                current_frame = {'frame_number': frame_number, 'depth_values': []}
            else:
                values = [float(value) for value in line.split()]
                current_frame['depth_values'].append(values)

    if current_frame is not None:
        frames.append(current_frame)

    return frames
