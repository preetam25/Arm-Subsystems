#!/usr/bin/env python
from math import pi, cos, sin

import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as GPIO
import time

class Arm:
    """docstring for ."""
    def __init__(self, velpin, pinH,pinL):
        self.VelPin=velpin
        GPIO.setup(self.VelPin,GPIO.OUT)
        self.ActuatorHigh = pinH
        self.ActuatorLow = pinL
        GPIO.setup(self.ActuatorHigh,GPIO.OUT)
        GPIO.setup(self.ActuatorLow,GPIO.OUT)
        self.actuator_velocity = 0
        self.Velocity=GPIO.PWM(self.VelPin,255)
        self.Velocity.start(0)


    def update(self):
        if self.actuator_velocity < -10:
            self.actuator_backward()
        elif self.actuator_velocity > 10:
            self.actuator_forward()
        else:
            self.actuator_stopped()

    def actuator_forward(self):
        GPIO.output(self.ActuatorHigh,GPIO.HIGH)
        GPIO.output(self.ActuatorLow,GPIO.LOW)
        self.Velocity.ChangeDutyCycle(abs(self.actuator_velocity))
        #time.sleep(0.02)

    def actuator_backward(self):
        GPIO.output(self.ActuatorHigh,GPIO.LOW)
        GPIO.output(self.ActuatorLow,GPIO.HIGH)
        self.Velocity.ChangeDutyCycle(abs(self.actuator_velocity))
        #time.sleep(0.02)

    def actuator_stopped(self):
        GPIO.output(self.ActuatorHigh,GPIO.LOW)
        GPIO.output(self.ActuatorLow,GPIO.LOW)
        self.Velocity.ChangeDutyCycle(0)
        #time.sleep(0.02)
