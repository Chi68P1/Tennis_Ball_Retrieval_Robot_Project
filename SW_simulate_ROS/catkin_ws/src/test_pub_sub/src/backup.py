#!/usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

priority = False
count1 = count2 = 0
error = 0
P = I = D = previousError = PIDvalue = lsp = rsp = 0
sample_time = 0.005

def find_object(img):
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #hsv_frame = cv2.resize(hsv_frame, (640, 300))

    low_H = 20
    low_S = 100
    low_V = 100
    high_H = 30
    high_S = 255
    high_V = 255
    #cv2.imshow("mask",hsv_frame)
    mask_frame = cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
    cv2.imshow("mask2", mask_frame)
    contours, hierarchy = cv2.findContours(mask_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    X, Y, W, H = 0, 0, 0, 0
    area_biggest = 0

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 15:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h > W * H:
                X, Y, W, H = x, y, w, h
                area_biggest = area
    img = cv2.rectangle(img, (X, Y), (X + W, Y + H), (0, 0, 255), 2)
    cx = X + W/2
    cy = Y + H/2

    cv2.imshow("window", img)
    cv2.waitKey(3)

    return cx,cy, area_biggest

def robot_control(cx,cy, area_biggest):
    global priority
    global count1,count2

    global P, I, D, previousError, PIDvalue, lsp, rsp,error, sample_time

    error = 320 - cx
    P = error
    I += error
    D = error - previousError

    Pvalue = 0.01 * P
    Ivalue = 0.00 * I
    Dvalue = 0.0001 * D / sample_time

    PIDvalue = Pvalue + Ivalue + Dvalue
    previousError = error

    lsp = -((640-cy)*0.02- PIDvalue)
    rsp = -((640-cy)*0.02+ PIDvalue)

    if lsp > 20:
        lsp = 20
    if lsp < -20:
        lsp = -20
    if rsp > 20:
        rsp = 20
    if rsp < -20:
        rsp = -20

    joint1 = rospy.Publisher('/my_TRR/joint1_velocity_controller/command', Float64, queue_size=10)
    joint2 = rospy.Publisher('/my_TRR/joint2_velocity_controller/command', Float64, queue_size=10)
    joint3 = rospy.Publisher('/my_TRR/joint3_velocity_controller/command', Float64, queue_size=10)
    joint4 = rospy.Publisher('/my_TRR/joint4_velocity_controller/command', Float64, queue_size=10)
    message1 = Float64()
    message2 = Float64()
    message3 = Float64()
    message4 = Float64()

    if (priority == True):
        if (count1 <50):
            text = "straight priority"
            message1 = min(-10,rsp) 
            message2 = min(-10,rsp) 
        else:
            text = "Done"
            priority = False
            count1 = 0
        count1 = count1 + 1
        rospy.sleep(0.001)

    elif (cy > 160)& (300 <cx <340):
        priority = True
        text = "straight priorit"

    elif cx > 0:
        count2 = 0
        text = "straight"
        message1 = rsp
        message2 = lsp

    else:
        priority = False    
        text = "searching"
        message1 = -5
        message2 = 5

        count2 = count2 + 1

    if (count2 <300):
        joint1.publish(message1)
        joint2.publish(message2)
        joint3.publish(-110)
        joint4.publish(-110)
    else:
        text = "Stop"
        joint1.publish(0)
        joint2.publish(0)
        joint3.publish(0)
        joint4.publish(0)

    print(text)
    print(area_biggest)
    print(cx)
    print(cy)
    print(message1)
    print(message2)
    print("")

def stop():
    joint1 = rospy.Publisher('/my_TRR/joint1_velocity_controller/command', Float64, queue_size=10)
    joint2 = rospy.Publisher('/my_TRR/joint2_velocity_controller/command', Float64, queue_size=10)
    message1 = 0
    message2 = 0
    joint1.publish(message1)
    joint2.publish(message2)
    print("stop")

def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    global image_received
    image_received = True
    image = cv_image

    cx,cy, area_biggest = find_object(cv_image)
    robot_control(cx,cy, area_biggest)

if __name__ == '__main__':
    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    bridge = CvBridge()
    image_received = False

    # Connect image topic
    img_topic = "/my_robot/camera1/image_raw"
    image_sub = rospy.Subscriber(img_topic, Image, callback)

    # Set the desired rate to achieve 5ms per callback
    rate = rospy.Rate(200)  # Tần suất 200 Hz (1/0.005)

    # Allow up to one second for connection
    rospy.sleep(1)

    while not rospy.is_shutdown():
        if image_received:
            # Xử lý hình ảnh và điều khiển robot
            image_received = False

        rate.sleep()
