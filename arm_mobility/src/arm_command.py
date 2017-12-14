#!/usr/bin/env python
from __future__ import division 
from math import pi, cos, sin
import diagnostic_msgs
import diagnostic_updater
from roboclaw import RoboClaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,Float32MultiArray,String
import time
import numpy as np
import thread
from arm_util import *
from scipy.interpolate import spline
from serial.serialutil import SerialException as SerialException

class SteerClaw:

    def __init__(self, address, dev_name, baud_rate, name, deadzone1 = 2, deadzone2 = 2, sample_time=0.1, last_time=0.00, current_time=0.00):
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
        0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
        0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
        0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
        0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
        0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
        0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
        0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
        0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
        0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
        0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
        0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
        0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
        0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
        0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
        0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
        0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        self.claw = RoboClaw(address, dev_name, baud_rate)
        self.name = name
        self.claw.ResetEncoders()
        self.targetRpm1=0
        self.targetRpm2=0
        self.deltax=0
        self.deltay=0
        self.sample_time = sample_time
        self.last_time = 0.00
        self.last_error1 = 0.00
        self.last_error2 = 0.00
        self.enc1Pos=0.00
        self.enc2Pos=0.00
        self.finalRpm1Val=0.00
        self.finalRpm2Val=0.00

    def pub_pot(self, pub, claw_name):
        pot_val1 = self.claw.ReadEncM1()[1]
        pot_val2 = self.claw.ReadEncM2()[1]
        pub.publish(claw_name+"| Pot1 Val :" +str(pot_val1) + "| Pot2 Val :" +str(pot_val2))

    def pub_curr(self,pub,claw_name):
        curr_val = self.claw.ReadCurrents()
        pub1.publish(claw_name+"| Curr1 Val :" +str(curr_val[1]) + "| Curr2 Val :" +str(curr_val[2]))

    def update_rpm(self):
        pot_val1=self.claw.ReadEncM1()[1]
        pot_val2=self.claw.ReadEncM2()[1]
        dt=0.001
        #Get Lengths from pot values
        (estimated_L1,estimated_L2)=get_act_lengths(pot_val1,pot_val2)
        curr_state=np.array([estimated_L1,estimated_L2])
        #Get Angles from pot measurement
        (estimated_theta,estimated_phi)=L_to_Angle(estimated_L1,estimated_L2)
        print("Current State: | " +str(estimated_theta)+" | "+str(estimated_phi))
        (target_theta,target_phi)=get_target_angles(estimated_theta,estimated_phi,self.deltax,self.deltay)
        if(target_theta==-1):
            print("Not Reachable")
            target_state=curr_state
        else:
            (target_L1,target_L2)=get_actuator_lengths(target_theta,target_phi)
            if(target_L1<25 or target_L2<25):
                print("Not Reachable")
                target_state=curr_state
            else:
                target_state=np.array([target_L1,target_L2])
        print("Net Displacement required "+str((target_state-curr_state)/2.54))
        deriv=(target_state-curr_state)/dt

        velM1=int(deriv[0])
        #velM1=int(230*self.deltax)
        if velM1>10:
            self.claw.ForwardM1(min(255, velM1))
        elif velM1<-10:
            self.claw.BackwardM1(min(255, -velM1))
        else:
            self.claw.ForwardM1(0)

        #velM2=int(230*self.deltay)
        velM2=int(deriv[1])
        if velM2>10:
            self.claw.ForwardM2(min(255, velM2))
        elif velM2<-10:
            self.claw.BackwardM2(min(255, -velM2))
        else:
            self.claw.ForwardM2(0)

def steer_callback(inp):
    actuator_lock = inp.data[0]  
    elbowmotor_lock = inp.data[1]    
    roboclaw1.deltax=0.5*actuator_lock
    roboclaw1.deltay=1.2*elbowmotor_lock


if __name__ == "__main__":

    rospy.init_node("roboclaw_node")
    rospy.loginfo("Starting Arm Close loop node")
    pub = rospy.Publisher('Pot_Val', String, queue_size=10)
    pub1 = rospy.Publisher('Curr_Val', String, queue_size=10)
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, steer_callback)
    r_time = rospy.Rate(1)


    for i in range(20):
        try:
            roboclaw1 = SteerClaw(0x80, "/dev/ttyACM0", 9600, "BaseClaw")
        except SerialException:
            rospy.logwarn("Could not connect to Arm RoboClaw1, retrying...")
            r_time.sleep()
    
    rospy.loginfo("Connected to Arm RoboClaw1")


    r_time = rospy.Rate(5)
    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)

    while not rospy.is_shutdown():
        roboclaw1.update_rpm()
        roboclaw1.pub_pot(pub,"roboclaw1")
        roboclaw1.pub_curr(pub,"roboclaw1")

        r_time.sleep()

    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    #roboclaw2.claw.ForwardM1(0)
    #roboclaw2.claw.ForwardM2(0)
