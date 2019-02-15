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
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String, Float64
import time
import numpy as np
import thread
from serial.serialutil import SerialException as SerialException
import signal
import sys

#-----------------------------------------------------------------
#SIGINT handler
def sigint_handler_arm(signal, frame):
    if(enable_actuator1claw):
        actuator1Claw.claw.ForwardM1(0)
        actuator1Claw.claw.ForwardM2(0)
    if(enable_actuator2claw):
        actuator2Claw.claw.ForwardM1(0)
        actuator2Claw.claw.ForwardM2(0)
    if(enable_motorclaw):
        motorclaw.claw.ForwardM1(0)
        motorclaw.claw.ForwardM2(0)
    sys.exit(0)

class ArmClaw:

    def __init__(self, address, dev_name, baud_rate, reset_encoders=False):
        self.claw = RoboClaw(address, dev_name, baud_rate)
        self.past_val=False
        if(reset_encoders):
            self.claw.ResetEncoders()
        
    def setactuator1Constants(self):
        #When these variables are set to +/- 1 M1, M2 move fwd bkwd
        self.actuator1clawM1=0
        self.actuator1clawM2=0

    def setactuator2Constants(self):
        #When these variables are set to +/- 1 M1, M2 move fwd bkwd
        self.actuator2clawM1=0
        self.actuator2clawM2=0    

    def setmotorConstants(self):
        #When grip roll is set to 1, we get ACW rotn o/w CW and base rotation is set to 1, we get ACW rotn o/w CW
    	self.grip_roll=0
    	self.BR_update=0

    def pub_pot(self, pub):
        #Read absolute encoder values and publish
        pot_val1 = self.claw.ReadEncM1()[1]
        pot_val2 = self.claw.ReadEncM2()[1]
        pot_val=Float64MultiArray(data=[pot_val1,pot_val2])
        pub.publish(pot_val)


    # Update Actuator1 motions
    def update_actuator1(self):
        #Make actuator1clawM1 move
        velM1=int(230*self.actuator1clawM1)
        if velM1>0:
            self.claw.ForwardM1(min(255, velM1))
        elif velM1<0:
            self.claw.BackwardM1(min(255, -velM1))
        else:
            self.claw.ForwardM1(0)

        #Make acctuator1clawM2 move
        velM2=int(230*self.actuator1clawM2)
        if velM2>10:
            self.claw.ForwardM2(min(255, velM2))
        elif velM2<-10:
            self.claw.BackwardM2(min(255, -velM2))
        else:
            self.claw.ForwardM2(0)

    # Update Actuator2 motions
    def update_actuator2(self):
        #Make actuator2clawM1 move
        velM1=int(230*self.actuator2clawM1)
        if velM1>0:
            self.claw.ForwardM1(min(255, velM1))
        elif velM1<0:
            self.claw.BackwardM1(min(255, -velM1))
        else:
            self.claw.ForwardM1(0)

        #Make acctuator2clawM2 move
        velM2=int(230*self.actuator2clawM2)
        if velM2>10:
            self.claw.ForwardM2(min(255, velM2))
        elif velM2<-10:
            self.claw.BackwardM2(min(255, -velM2))
        else:
            self.claw.ForwardM2(0)

    def update_motor(self):
        
        #Make base motor move
<<<<<<< HEAD
        if (self.past_val):
            # print("Laser on")
            self.claw.BackwardM1(255)
        else:
            # print("Laser off")
            self.claw.BackwardM1(0) 
=======
        velM1=int(230*self.BR_update)
        if velM1>10:
            print("Vel M1 commanded")
            self.claw.ForwardM1(min(255, velM1))
        elif velM1<-10:
            print("Vel M1 commanded backward")
            self.claw.BackwardM1(min(255,-velM1))
        else:
            self.claw.ForwardM1(0) 
>>>>>>> 85d6927b5cd9f687fa65ac1d401f4e954f9ee978

        #Make the gripper roll
        velM2=int(230*self.grip_roll)
        if velM2>10:
            print("Vel M2 commanded")
            self.claw.ForwardM2(min(255, velM2))
        elif velM2<-10:
            print("Vel M2 commanded")
            self.claw.BackwardM2(min(255, -velM2))
        else:
            self.claw.ForwardM2(0)

def arm_callback(inp):
    if(enable_actuator1claw):
        # print(inp.data[1])
        actuator1Claw.actuator1clawM1=inp.data[1]
        actuator1Claw.actuator1clawM2=inp.data[2]
    
    if(enable_actuator2claw):
        actuator2Claw.actuator2clawM1=inp.data[3]
        actuator2Claw.actuator2clawM2=inp.data[4]
    
    if(enable_motorclaw):
    	motorclaw.grip_roll=inp.data[5]
<<<<<<< HEAD
        print(inp.data[0])
        if(inp.data[0]==1):
            motorclaw.past_val= not motorclaw.past_val
=======
        motorclaw.BR_update=-inp.data[0]
>>>>>>> 85d6927b5cd9f687fa65ac1d401f4e954f9ee978
    
    if(inp.data[0]!=0):
        print("Base Rotn commanded")
    if(inp.data[1]!=0):
        print("Shoulder Actuator commanded, M1 commanded")
    if(inp.data[2]!=0):
        print("Elbow Actuator commanded, M2 commanded")
    if(inp.data[3]!=0):
        print("Wrist Actuator commanded, M1 commanded")    
    if(inp.data[4]!=0):
        print("Finger Actuator commanded, M2 commanded")
    if(inp.data[5]!=0):
        print("Gripper Roll commanded")
<<<<<<< HEAD
=======


def GUI_callback(inp):
    print(inp.data)
    print("Commanded")
    if(inp.data=="Ac1_Up"):
        actuator1Claw.actuator1clawM1=1
    if(inp.data=="Ac1_Down"):
        actuator1Claw.actuator1clawM1=-1
    if(inp.data=="Ac2_Up"):
        actuator1Claw.actuator1clawM2=1
    if(inp.data=="Ac2_Down"):
        actuator1Claw.actuator1clawM2=-1
    if(inp.data=="Ac3_Up"):
        actuator2Claw.actuator2clawM1=1
    if(inp.data=="Ac3_Down"):
        actuator2Claw.actuator2clawM1=-1
    if(inp.data=="Ac4_Up"):
        actuator2Claw.actuator2clawM2=1
    if(inp.data=="Ac4_Down"):
        actuator2Claw.actuator2clawM2=-1
    if(inp.data=="Grip_Up"):
        motorclaw.grip_roll=1
    if(inp.data=="Grip_Down"):
        motorclaw.grip_roll=-1
    if(inp.data=="Stop_All"):
        actuator1Claw.actuator1clawM1=0
        actuator1Claw.actuator1clawM2=0
        actuator2Claw.actuator2clawM1=0
        actuator2Claw.actuator2clawM2=0
        motorclaw.grip_roll=0
        motorclaw.BR_update=0

>>>>>>> 85d6927b5cd9f687fa65ac1d401f4e954f9ee978

# def cont_callback(inp):
#     actuator1Claw.actuator1clawM1=inp.data[0]
#     actuator1Claw.actuator1clawM2=inp.data[1]

def GUI_callback(inp):
    print(inp.data)
    print("Commanded")
    if(inp.data=="Ac1_Up"):
        actuator1Claw.actuator1clawM1=1
    if(inp.data=="Ac1_Down"):
        actuator1Claw.actuator1clawM1=-1
    if(inp.data=="Ac2_Up"):
        actuator1Claw.actuator1clawM2=1
    if(inp.data=="Ac2_Down"):
        actuator1Claw.actuator1clawM2=-1
    if(inp.data=="Ac3_Up"):
        actuator2Claw.actuator2clawM1=1
    if(inp.data=="Ac3_Down"):
        actuator2Claw.actuator2clawM1=-1
    if(inp.data=="Ac4_Up"):
        actuator2Claw.actuator2clawM2=1
    if(inp.data=="Ac4_Down"):
        actuator2Claw.actuator2clawM2=-1
    if(inp.data=="Grip_Up"):
        motorclaw.grip_roll=1
    if(inp.data=="Grip_Down"):
        motorclaw.grip_roll=-1
    if(inp.data=="Stop_All"):
        actuator1Claw.actuator1clawM1=0
        actuator1Claw.actuator1clawM2=0
        actuator2Claw.actuator2clawM1=0
        actuator2Claw.actuator2clawM2=0
        motorclaw.grip_roll=0
        motorclaw.BR_update=0


# def cont_callback(inp):
#     actuator1Claw.actuator1clawM1=inp.data[0]
#     actuator1Claw.actuator1clawM2=inp.data[1]

if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler_arm)
    rospy.init_node("Arm Roboclaw_node")
    rospy.loginfo("Starting Arm Open loop node")
<<<<<<< HEAD
    pub_pot1 = rospy.Publisher('Pot_Val_Claw1', Float64MultiArray, queue_size=10)
    pub_pot2 = rospy.Publisher('Pot_Val_Claw2', Float64MultiArray, queue_size=10)
=======
    pub_pot = rospy.Publisher('Pot_Val', Float64MultiArray, queue_size=10)
>>>>>>> 85d6927b5cd9f687fa65ac1d401f4e954f9ee978
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, arm_callback)
    rospy.Subscriber("Django_node", String, GUI_callback)
    # joystick=True

    # if(joystick==True):
    #     rospy.Subscriber("/rover/arm_directives", Float64MultiArray, arm_callback)
    # else:
    #     rospy.Subscriber("arm_cont", Float64MultiArray, cont_callback)                                

    r_time = rospy.Rate(1)
    enable_actuator1claw=True
    enable_actuator2claw=True
    enable_motorclaw=True	
    
    if(enable_actuator1claw):
        for i in range(20):
            try:
                actuator1Claw = ArmClaw(0x80, "/dev/actuator1Claw", 9600)
                actuator1Claw.setactuator1Constants()
            except SerialException:
                rospy.logwarn("Could not connect to Actuator1 Claw, retrying...")
                r_time.sleep()
        rospy.loginfo("Connected to Actuator1 Claw")

    if(enable_actuator2claw):
        for i in range(20):
            try:
                actuator2Claw = ArmClaw(0x80, "/dev/actuator2Claw", 9600)
                actuator2Claw.setactuator2Constants()
            except SerialException:
                rospy.logwarn("Could not connect to Actuator2 Claw, retrying...")
                r_time.sleep()
        rospy.loginfo("Connected to Actuator2 Claw")    

    if(enable_motorclaw):
        for i in range(20):
            try:
                motorclaw = ArmClaw(0x80, "/dev/motorClaw", 9600)
                motorclaw.setmotorConstants()

            except SerialException:
                rospy.logwarn("Could not connect to motor Claw, retrying...")
                r_time.sleep()

        rospy.loginfo("Connected to motor Claw")


    r_time = rospy.Rate(5)
    # Initialize the motors and actuator !!!VVV IMP!!!!
    if(enable_actuator1claw):
        actuator1Claw.claw.ForwardM1(0)
        actuator1Claw.claw.ForwardM2(0)
    if(enable_actuator2claw):
        actuator2Claw.claw.ForwardM1(0)
        actuator2Claw.claw.ForwardM2(0)
    if(enable_motorclaw):
        motorclaw.claw.ForwardM1(0)
        motorclaw.claw.ForwardM2(0)

    while True:
        try:
            if(enable_actuator1claw):
                actuator1Claw.update_actuator1()
<<<<<<< HEAD
                actuator1Claw.pub_pot(pub_pot1)
            if(enable_actuator2claw):
                actuator2Claw.update_actuator2()
                actuator2Claw.pub_pot(pub_pot2)
=======
                actuator1Claw.pub_pot(pub_pot)
            if(enable_actuator2claw):
                actuator2Claw.update_actuator2()
                actuator2Claw.pub_pot(pub_pot)
>>>>>>> 85d6927b5cd9f687fa65ac1d401f4e954f9ee978
            if(enable_motorclaw):
                motorclaw.update_motor()
            r_time.sleep()

        except SerialException:
            print("Serial Exception Raised, telling motors to stop")
            if(enable_actuator1claw):
                for i in range(20):
                    try:
                        print("in try...")
                        actuator1Claw = ArmClaw(0x80, "/dev/actuator1Claw", 9600)
                        actuator1Claw.setactuator1Constants()
                        
                    except SerialException:
                        print("in except...")
                        rospy.logwarn("Could not re-connect to Actuator1 Claw, retrying...")  
                        r_time.sleep()
                        
                rospy.loginfo("Re-Connected to Actuator1 Claw")
                actuator1Claw.claw.ForwardM1(0)
                actuator1Claw.claw.ForwardM2(0)
            # print("Motors should be stopped by kill switch")
            # sys.exit(00)


