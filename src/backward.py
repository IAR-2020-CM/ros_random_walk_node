#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rospy
from geometry_msgs.msg import Twist
import numpy
from sensor_msgs.msg import LaserScan, PointCloud2
import sys, tty, termios
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import cv2.cv as cv
import math
import random

dist = []

def random_sign():
    if random.random() < 0.5:
        return 1
    else:
        return -1

def cmd_vel_speed(linear_cmd,
                  angular_cmd=0,
                  coef_linear_speed=1,
                  coef_angular_speed=0.5):

    speed = Twist()
    speed.linear.x = linear_cmd * coef_linear_speed
    speed.angular.z = angular_cmd * coef_angular_speed
    return speed

def camera_callback(msg):
    global dist
    scan = msg.ranges
    if len(scan) != 0:
        dist = scan

    #try:
    #    depth_image = bridge.imgmsg_to_cv(msg, "32FC1")
    #    dl = depth_image[depth_image.height // 2, :]
    #    depth_line = [dl[0, i] for i in range(dl.width) if not numpy.isnan(dl[0, i])]
    #    rospy.loginfo("Line "  + str(min(depth_line)) )
    #except CvBridgeError, e:
    #    print e


def autonomous_move():
    global dist

    rospy.init_node('autonomous_move')
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

    rospy.Subscriber('/scan', LaserScan, camera_callback)

    r = rospy.Rate(10)
    obstacle = False

    while not rospy.is_shutdown():
        s = "hello wold %s" % rospy.get_time()
        #rospy.loginfo(s)

        if any(not numpy.isnan(d) and d < 1 for d in dist):
            linear = 0
            if not obstacle:
                angular = random_sign()
            obstacle = True
        else:
            linear = -0.1
            angular = 0
            obstacle = False

        speed = cmd_vel_speed(linear_cmd=linear, angular_cmd=angular)
        pub.publish(speed)

        r.sleep()

if __name__== '__main__':

    try:
        autonomous_move()
    except rospy.ROSInterruptException:
        pass


