#!/usr/bin/env python
from math import pi, cos, sin
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from Class_Arm import Arm
import time
import RPi.GPIO as GPIO
from multiprocessing import Process

OL_motors = Arm()

def arm_ol_callback(inp):
    OL_motors.rover_velocity1 = inp.data[4]*100;
    OL_motors.rover_velocity2 = inp.data[5]*100;

if __name__ == "__main__":
    rospy.init_node("Arm_node")
    rospy.loginfo("Starting Arm Open Loop Node")
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, arm_ol_callback)

    while not rospy.is_shutdown():
        OL_motors.fingers_update()
        OL_motors.wrist_update()

    GPIO.cleanup()
