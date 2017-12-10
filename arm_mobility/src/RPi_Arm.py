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

GPIO.setmode(GPIO.BCM)
Actuator1 = Arm(18,17,27)
Actuator2 = Arm(12,6,5)


def drive_callback(inp):
    Actuator1.rover_velocity = inp.data[1]*255
    Actuator2.rover_velocity = inp.data[2]*255

if __name__ == "__main__":
    rospy.init_node("Actuator_node")
    rospy.loginfo("Starting Actuator Node")
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, drive_callback)

    while not rospy.is_shutdown():
        p1 = Process(target=Actuator1.update())
        p2 = Process(target=Actuator2.update())
        p1.start()
        p2.start()
        p1.join()        
        p2.join()

    GPIO.cleanup()
