#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent
import numpy as np
import math
import random

dist = []
bumpers = set()

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

def bumper_callback(msg):
    global bumpers
    if msg.state == 1:
        bumpers.add(msg.bumper)
    else:
        bumpers.remove(msg.bumper)

def autonomous_move():
    global dist

    rospy.init_node('autonomous_move')
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

    rospy.Subscriber('/scan', LaserScan, camera_callback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)

    r = rospy.Rate(10)
    obstacle = False

    while not rospy.is_shutdown():
        s = "hello wold %s" % rospy.get_time()
        #rospy.loginfo(s)

        if any(not np.isnan(d) and d < 1 for d in dist):
            linear = 0
            if not obstacle:
                angular = random_sign()
            obstacle = True
        elif len(bumpers) == 0:
            linear = -0.1
            angular = 0
            obstacle = False
        if len(bumpers) != 0:
            linear = 0
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


