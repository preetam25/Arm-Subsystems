#!/usr/bin/env python
from __future__ import division 
from math import pi, cos, sin
import diagnostic_msgs
import diagnostic_updater
from roboclaw import RoboClaw
import rospy
import tf
from geometry_msgs-.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,Float32MultiArray,String, Float64
import time
import numpy as np
import thread
from serial.serialutil import SerialException as SerialException
import signal
import sys

#-----------------------------------------------------------------
#SIGINT handler
def sigint_handler_arm(signal, frame):
    if(enable_actuatorclaw):
        actuatorClaw.claw.ForwardM1(0)
        actuatorClaw.claw.ForwardM2(0)
    if(enable_wristclaw):
        wristClaw.claw.ForwardM1(0)
        wristClaw.claw.ForwardM2(0)
    if(enable_gripperclaw):
        gripperClaw.claw.ForwardM1(0)
        gripperClaw.claw.ForwardM2(0)
    sys.exit(0)

class ArmClaw:

    def __init__(self, address, dev_name, baud_rate, reset_encoders=False):
        self.claw = RoboClaw(address, dev_name, baud_rate)
        if(reset_encoders):
            self.claw.ResetEncoders()
        
    def setActuatorConstants(self):
        #When these variables are set to +/- 1 M1, M2 move fwd bkwd
        self.actuatorclawM1=0
        self.actuatorclawM2=0

    def setGripperConstants(self):
        #When grip roll is set to 1, we get ACW rotn o/w CW. Fing_close=1 means fingers will be closed
    	self.grip_roll=0
    	self.fing_close=0

    def pub_pot(self, pub):
        #Read absolute encoder values and publish
        pot_val1 = self.claw.ReadEncM1()[1]
        pot_val2 = self.claw.ReadEncM2()[1]
        pot_val=Float64MultiArray(data=[pot_val1,pot_val2])
        pub.publish(pot_val)

    def pub_enc(self, pub):
        #Read quadrature encoder values
        enc_val = self.claw.ReadEncM2()[1]
        pub.publish(enc_val)

    # Update Actuator motions
    def update_actuators(self):
        #Make actuatorclawM1 move
        velM1=int(230*self.actuatorclawM1)
        if velM1>0:
            self.claw.ForwardM1(min(255, velM1))
        elif velM1<0:
            self.claw.BackwardM1(min(255, -velM1))
        else:
            self.claw.ForwardM1(0)

        #Make acctuatorclawM2 move
        velM2=int(230*self.actuatorclawM2)
        if velM2>10:
            self.claw.ForwardM2(min(255, velM2))
        elif velM2<-10:
            self.claw.BackwardM2(min(255, -velM2))
        else:
            self.claw.ForwardM2(0)

    def update_gripper(self):
        # Close/Open the fingers
        velM1=int(230*self.fing_close)
        if velM1>10:
            self.claw.ForwardM1(min(255, velM1))
        elif velM1<-10:
            self.claw.BackwardM1(min(255, -velM1))
        else:
            self.claw.ForwardM1(0)
        
        #Make the gripper roll
        velM2=int(230*self.grip_roll)
        if velM2>10:
            self.claw.ForwardM2(min(255, velM2))
        elif velM2<-10:
            self.claw.BackwardM2(min(255, -velM2))
        else:
            self.claw.ForwardM2(0)
'''
    def setPIDconstants(self,Kp,Ki,Kd,Kdd):
        self.kp=Kp
        self.ki=Ki
        self.kd=Kd
        self.kdd=Kdd
        self.mode="lock"
        self.RPM_update=0
        self.clmotor_speed=40
        self.last_error1 = 0.00
        self.enc1Pos=0.00
        self.last_error1 = 0.00
        self.PTerm1=0.00
        self.ITerm1=0.00
        self.DTerm1=0.00
        self.DDTerm1=0.00
        self.prev_Dterm=0.00
        self.delta_error1=0.00
        self.diff1=0.00
        self.enc1Pos=0.00
        self.lockEnc1Val=0.00
        self.initialAngleM1=0
        self.BR_update=0
        self.int_windout1=10
        self.deadzone1=5

    def update_pid(self):
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        #----------------------------------------------------
        #Roboclaw1
        if ((delta_time >= self.sample_time) and (self.mode=="lock")):
            self.enc1Pos = self.claw.ReadEncM2()[1]
            self.diff1 = self.lockEnc1Val - self.enc1Pos  #Error in 1
            
            self.delta_error1 = self.diff1 - self.last_error1
            self.PTerm1 = self.diff1 #Pterm
            self.ITerm1+=self.diff1*delta_time
            
            if (self.ITerm1 < -self.int_windout1):
                self.ITerm1 = -self.int_windout1
            elif (self.ITerm1 > self.int_windout1):
                self.ITerm1 = self.int_windout1

            self.DTerm1 = self.delta_error1 / delta_time
            self.DDTerm1=(self.DTerm1-self.prev_Dterm)/delta_time
            # Remember last time and last error for next calculation
            self.last_error1 = self.diff1
            self.last_Ierr1=self.ITerm1
            self.prev_Dterm=self.DTerm1

            #velM1 = int((self.kp*self.PTerm1) + (self.ki * self.ITerm1) + (self.kd * self.DTerm1)+(self.kdd*self.DDTerm1))
            velM1=0
            if self.enc1Pos < (self.lockEnc1Val - self.deadzone1):
                self.claw.BackwardM2(min(255, velM1))
            elif self.enc1Pos > (self.lockEnc1Val + self.deadzone1):
                self.claw.ForwardM2(min(255, -velM1))
            else:
                self.claw.ForwardM2(0)

        if(self.mode=="roll"):
            velM1=self.RPM_update
            #velM1=0
            if velM1>0:
                self.claw.ForwardM2(min(255, velM1))
            elif velM1<0:
                self.claw.BackwardM2(min(255, -velM1))
            else:
                self.claw.ForwardM2(0)
            self.lockEnc1Val=self.claw.ReadEncM2()[1]
            print("Locking to position:",str(self.lockEnc1Val))

        velM2=int(230*self.BR_update)
        if velM2>10:
            self.claw.ForwardM1(min(255, velM2))
        elif velM2<-10:
            self.claw.BackwardM1(min(255, -velM2))
        else:
            self.claw.ForwardM1(0)
'''


def arm_callback(inp):
    if(enable_actuatorclaw):
        actuatorClaw.actuatorclawM1=inp.data[1]
        actuatorClaw.actuatorclawM2=inp.data[2]
    if(enable_wristclaw):
        wristClaw.BR_update=-inp.data[0]
        if(inp.data[3]==0):
            wristClaw.mode="lock"
        else:
            print("Entering Roll Mode")
            wristClaw.mode="roll"
            if(inp.data[3]==1):
                wristClaw.RPM_update=wristClaw.clmotor_speed
            else:
                wristClaw.RPM_update=-wristClaw.clmotor_speed
    
    if(enable_gripperclaw):
        gripperClaw.fing_close=inp.data[4]
        gripperClaw.grip_roll=inp.data[5]
    
    if(inp.data[0]!=0):
        print("Base Rotn commanded")
    if(inp.data[1]!=0):
        print("Shoulder Actuator commanded, M1 commanded")
    if(inp.data[2]!=0):
        print("Elbow Actuator commanded, M2 commanded")
    if(inp.data[4]!=0):
        print("Gripper Fingers commanded")
    if(inp.data[5]!=0):
        print("Gripper Roll commanded")   


if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler_arm)
    rospy.init_node("Arm Roboclaw_node")
    rospy.loginfo("Starting Arm Open loop node")
    pub_pot = rospy.Publisher('Pot_Val', Float64MultiArray, queue_size=10)
    pub_enc = rospy.Publisher('Enc_Val', Float64, queue_size=10)
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, arm_callback)
    r_time = rospy.Rate(1)
    enable_actuatorclaw=True
    enable_wristclaw=False
    enable_gripperclaw=False
    if(enable_actuatorclaw):
        for i in range(20):
            try:
                actuatorClaw = ArmClaw(0x80, "/dev/actuatorClaw", 9600)
                actuatorClaw.setActuatorConstants()
            except SerialException:
                rospy.logwarn("Could not connect to Actuator Claw, retrying...")
                r_time.sleep()
        rospy.loginfo("Connected to Actuator Claw")

    if(enable_wristclaw):
        for i in range(20):
            try:
                wristClaw = ArmClaw(0x81, "/dev/wristClaw", 9600)
                wristClaw.setPIDconstants(0.03,0.02,30,20)
            except SerialException:
                rospy.logwarn("Could not connect to Wrist Claw, retrying...")
                r_time.sleep()
    
    	rospy.loginfo("Connected to Wrist Claw")
    
    if(enable_gripperclaw):
        for i in range(20):
            try:
                gripperClaw = ArmClaw(0x80, "/dev/gripperClaw", 9600)
                gripperClaw.setGripperConstants()

            except SerialException:
                rospy.logwarn("Could not connect to Gripper Claw, retrying...")
                r_time.sleep()

        rospy.loginfo("Connected to Gripper Claw")


    r_time = rospy.Rate(5)
    # Initialize the motors and actuator !!!VVV IMP!!!!
    if(enable_actuatorclaw):
        actuatorClaw.claw.ForwardM1(0)
        actuatorClaw.claw.ForwardM2(0)
    if(enable_wristclaw):
        wristClaw.claw.ForwardM1(0)
        wristClaw.claw.ForwardM2(0)
    if(enable_gripperclaw):
        gripperClaw.claw.ForwardM1(0)
        gripperClaw.claw.ForwardM2(0)

    while True:
        if(enable_actuatorclaw):
            actuatorClaw.update_actuators()
            actuatorClaw.pub_pot(pub_pot)
        if(enable_wristclaw):
            wristClaw.update_pid()
            wristClaw.pub_enc(pub_end)
        if(enable_gripperclaw):
            gripperClaw.update_gripper()
        r_time.sleep()

    if(enable_actuatorclaw):
        actuatorClaw.claw.ForwardM1(0)
        actuatorClaw.claw.ForwardM2(0)
    if(enable_wristclaw):
        wristClaw.claw.ForwardM1(0)
        wristClaw.claw.ForwardM2(0)
    if(enable_gripperclaw):
        gripperClaw.claw.ForwardM1(0)
        gripperClaw.claw.ForwardM2(0)
