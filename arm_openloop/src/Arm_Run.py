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

Actuators = Arm()

def arm_ol_callback(inp):
    Actuators.rover_velocity1 = inp.data[0]*100;
    Actuators.rover_velocity2 = inp.data[1]*100;
    Actuators.rover_velocity3 = inp.data[2]*100;

if __name__ == "__main__":
    rospy.init_node("Arm_node")
    rospy.loginfo("Starting Arm Open Loop Node")
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, drive_callback)

    while not rospy.is_shutdown():
        Actuators.shoulder_update()
        Actuators.elbow_update()
        Actuators.BR_update()

    GPIO.cleanup()
