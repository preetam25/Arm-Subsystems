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

ShoulderHigh = 17
ShoulderLow = 27
ElbowHigh = 6
ElbowLow = 5

GPIO.setup(ShoulderHigh,GPIO.OUT)
GPIO.setup(ShoulderLow,GPIO.OUT)
GPIO.setup(ElbowHigh,GPIO.OUT)
GPIO.setup(ElbowLow,GPIO.OUT)



#Analog Pin Setup

VelPin1 = 18
VelPin2 = 12


class Drive:
    """docstring for ."""
    def __init__(self, motor_no):

        self.ShoulderPin=VelPin1
	GPIO.setup(self.ShoulderPin,GPIO.OUT)
	self.ElbowPin=VelPin2
        GPIO.setup(self.ElbowPin,GPIO.OUT)
        self.rover_velocity1 = 0
	self.rover_velocity2 = 0
        self.Velocity1=GPIO.PWM(self.ShoulderPin,255)
	self.Velocity2=GPIO.PWM(self.ElbowPin,255)
        self.Velocity1.start(0)
	self.Velocity2.start(0)


    def shoulder_update(self):
        if self.rover_velocity1 < -10:
            self.shoulder_forward()
        elif self.rover_velocity1 > 10:
            self.shoulder_backward()
        else:
            self.shoulder_stopped()

    def shoulder_forward(self):
        GPIO.output(ShoulderHigh,GPIO.HIGH)
        GPIO.output(ShoulderLow,GPIO.LOW)
        self.Velocity1.ChangeDutyCycle(abs(self.rover_velocity1))
        #time.sleep(0.02)

    def shoulder_backward(self):
        GPIO.output(ShoulderHigh,GPIO.LOW)
        GPIO.output(ShoulderLow,GPIO.HIGH)
        self.Velocity1.ChangeDutyCycle(abs(self.rover_velocity1))
        #time.sleep(0.02)

    def shoulder_stopped(self):
        GPIO.output(ShoulderHigh,GPIO.LOW)
        GPIO.output(ShoulderLow,GPIO.LOW)
        self.Velocity1.ChangeDutyCycle(0)
        #time.sleep(0.02)

    def elbow_update(self):
        if self.rover_velocity2 < -10:
            self.elbow_forward()
        elif self.rover_velocity2 > 10:
            self.elbow_backward()
        else:
            self.elbow_stopped()

    def elbow_forward(self):
        GPIO.output(ElbowHigh,GPIO.HIGH)
        GPIO.output(ElbowLow,GPIO.LOW)
        self.Velocity2.ChangeDutyCycle(abs(self.rover_velocity2))
        #time.sleep(0.02)

    def elbow_backward(self):
        GPIO.output(ElbowHigh,GPIO.LOW)
        GPIO.output(ElbowLow,GPIO.HIGH)
        self.Velocity2.ChangeDutyCycle(abs(self.rover_velocity2))
        #time.sleep(0.02)

    def elbow_stopped(self):
        GPIO.output(ElbowHigh,GPIO.LOW)
        GPIO.output(ElbowLow,GPIO.LOW)
        self.Velocity2.ChangeDutyCycle(0)
        #time.sleep(0.02)
