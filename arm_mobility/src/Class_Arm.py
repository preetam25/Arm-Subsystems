#!/usr/bin/env python
from math import pi, cos, sin

import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
#Digital Pins Setup
WristHigh = 17
WristLow = 27
FingerHigh = 6
FingerLow = 5
#Analog Pin Setup
VelPin1 = 18
VelPin2 = 12

GPIO.setup(WristHigh,GPIO.OUT)
GPIO.setup(WristLow,GPIO.OUT)
GPIO.setup(FingerHigh,GPIO.OUT)
GPIO.setup(FingerLow,GPIO.OUT)

class Arm:
    """docstring for ."""
    def __init__(self):
        self.WristPin=VelPin1
    	GPIO.setup(self.WristPin,GPIO.OUT)
    	self.FingerPin=VelPin2
        GPIO.setup(self.FingerPin,GPIO.OUT)
        self.rover_velocity1 = 0
    	self.rover_velocity2 = 0
        self.Velocity1=GPIO.PWM(self.WristPin,255)
    	self.Velocity2=GPIO.PWM(self.FingerPin,255)
        self.Velocity1.start(0)
    	self.Velocity2.start(0)

    def fingers_update(self):
        if self.rover_velocity1 < -10:
            self.fingers_forward()
        elif self.rover_velocity1 > 10:
            self.fingers_backward()
        else:
            self.fingers_stopped()

    def fingers_forward(self):
        GPIO.output(WristHigh,GPIO.HIGH)
        GPIO.output(WristLow,GPIO.LOW)
        self.Velocity1.ChangeDutyCycle(abs(self.rover_velocity1))
        #time.sleep(0.02)

    def fingers_backward(self):
        GPIO.output(WristHigh,GPIO.LOW)
        GPIO.output(WristLow,GPIO.HIGH)
        self.Velocity1.ChangeDutyCycle(abs(self.rover_velocity1))
        #time.sleep(0.02)

    def fingers_stopped(self):
        GPIO.output(WristHigh,GPIO.LOW)
        GPIO.output(WristLow,GPIO.LOW)
        self.Velocity1.ChangeDutyCycle(0)
        #time.sleep(0.02)

    def wrist_update(self):
        if self.rover_velocity2 < -10:
            self.wrist_forward()
        elif self.rover_velocity2 > 10:
            self.wrist_backward()
        else:
            self.wrist_stopped()

    def wrist_forward(self):
        GPIO.output(FingerHigh,GPIO.HIGH)
        GPIO.output(FingerLow,GPIO.LOW)
        self.Velocity2.ChangeDutyCycle(abs(self.rover_velocity2))
        #time.sleep(0.02)

    def wrist_backward(self):
        GPIO.output(FingerHigh,GPIO.LOW)
        GPIO.output(FingerLow,GPIO.HIGH)
        self.Velocity2.ChangeDutyCycle(abs(self.rover_velocity2))
        #time.sleep(0.02)

    def wrist_stopped(self):
        GPIO.output(FingerHigh,GPIO.LOW)
        GPIO.output(FingerLow,GPIO.LOW)
        self.Velocity2.ChangeDutyCycle(0)
        #time.sleep(0.02)