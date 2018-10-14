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
from std_msgs.msg import Float64MultiArray,Float32MultiArray,String, Float64
import time
import numpy as np
import thread
from serial.serialutil import SerialException as SerialException
import signal
import sys

#-----------------------------------------------------------------
#SIGINT handler
def sigint_handler(signal, frame):
    #print("here")
    global roboclaw1
    roboclaw1.deltay=0
    roboclaw1.deltax=0
    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    roboclaw2.claw.ForwardM1(0)
    roboclaw2.claw.ForwardM2(0)
    time.sleep(1)
    sys.exit(0)

class SteerClaw:

    def __init__(self, address, dev_name, baud_rate, name, sample_time=0.1, last_time=0.00, current_time=0.00):
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
        self.sample_time = sample_time
        self.last_time = 0.00
        self.current_time=0.00
        
    def setActuatorConstants(self):
        self.targetRpm1=0
        self.targetRpm2=0
        self.deltax=0
        self.deltay=0

    def pub_pot(self, pub):
        pot_val1 = self.claw.ReadEncM1()[1]
        pot_val2 = self.claw.ReadEncM2()[1]
        pot_val=Float64MultiArray(data=[pot_val1,pot_val2])
        pub.publish(pot_val)

    def pub_enc(self, pub):
        enc_val = self.claw.ReadEncM1()[1]
        pub.publish(enc_val)

    def pub_curr(self,pub):
        curr_val = self.claw.ReadCurrents()
        pub1.publish("Actuator Curr1 Val :" +str(curr_val[1]) + "| Curr2 Val :" +str(curr_val[2]))

    def update_rpm(self):

        #velM1=int(deriv[0])
        velM1=int(230*self.deltax)
        if velM1>10:
            self.claw.ForwardM1(min(255, velM1))
        elif velM1<-10:
            self.claw.BackwardM1(min(255, -velM1))
        else:
            self.claw.ForwardM1(0)

        #velM2=int(deriv[1])
        velM2=int(230*self.deltay)
        if velM2>10:
            self.claw.ForwardM2(min(255, velM2))
        elif velM2<-10:
            self.claw.BackwardM2(min(255, -velM2))
        else:
            self.claw.ForwardM2(0)

    def setPIDconstants(self,Kp,Ki,Kd):
        self.kp=Kp
        self.ki=Ki
        self.kd=Kd
        self.mode="lock"
        self.RPM_update=0
        self.clmotor_speed=0
        self.last_error1 = 0.00
        self.enc1Pos=0.00
        self.last_error1 = 0.00
        self.PTerm1=0.00
        self.ITerm1=0.00
        self.DTerm1=0.00
        self.diff_ITerm1=0.00
        self.last_Ierr1=0.00
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
            self.enc1Pos = self.claw.ReadEncM1()[1]
            self.diff1 = self.lockEnc1Val - self.enc1Pos  #Error in 1
            
            self.delta_error1 = self.diff1 - self.last_error1
            self.PTerm1 = self.diff1 #Pterm
            self.ITerm1+=self.diff1*delta_time
            
            if(self.last_Ierr1!=0):
                self.diff_ITerm1=self.ITerm1-self.last_Ierr1
            
            if (self.ITerm1 < -self.int_windout1):
                self.ITerm1 = -self.int_windout1
            elif (self.ITerm1 > self.int_windout1):
                self.ITerm1 = self.int_windout1
            self.DTerm1 = self.delta_error1 / delta_time
            # Remember last time and last error for next calculation
            self.last_error1 = self.diff1
            self.last_Ierr1=self.ITerm1

            #velM1 = int((self.kp*self.PTerm1) + (self.ki * self.ITerm1) + (self.kd * self.DTerm1))
            velM1=0
            if self.enc1Pos < (self.lockEnc1Val - self.deadzone1):
                self.claw.BackwardM1(min(255, velM1))
            elif self.enc1Pos > (self.lockEnc1Val + self.deadzone1):
                self.claw.ForwardM1(min(255, -velM1))
            else:
                self.claw.ForwardM1(0)

        if(self.mode=="roll"):
            #velM1=self.RPM_update
            velM1=0
            if velM1>0:
                self.claw.ForwardM1(min(255, velM1))
            elif velM1<0:
                self.claw.BackwardM1(min(255, -velM1))
            else:
                self.claw.ForwardM1(0)
            self.lockEnc1Val=self.claw.ReadEncM1()[1]
            print("Locking to position:",str(self.lockEnc1Val))

        velM2=int(230*self.BR_update)
        if velM2>10:
            self.claw.ForwardM2(min(255, velM2))
        elif velM2<-10:
            self.claw.BackwardM2(min(255, -velM2))
        else:
            self.claw.ForwardM2(0)



def arm_callback(inp):    
    #roboclaw2.BR_update=inp.data[0]
    roboclaw1.deltay=inp.data[0]
    roboclaw1.deltax=inp.data[1]
    # if(inp.data[3]==0):
    #     print("Lock Mode")
    #     roboclaw2.mode="lock"
    # else:
    #     print("Entering Roll")
    #     roboclaw2.mode="roll"
    #     if(inp.data[3]==1):
    #         roboclaw2.RPM_update=roboclaw2.clmotor_speed
    #     else:
    #         roboclaw2.RPM_update=-roboclaw2.clmotor_speed   


if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node("roboclaw_node")
    rospy.loginfo("Starting Arm Close loop node")
    pub = rospy.Publisher('Pot_Val', Float64MultiArray, queue_size=10)
    pub1 = rospy.Publisher('Curr_Val', String, queue_size=10)
    pub2 = rospy.Publisher('Enc_Val', Float64, queue_size=10)
    rospy.Subscriber('arm_cont', Float64MultiArray, arm_callback)
    #rospy.Subscriber("/rover/arm_directives", Float64MultiArray, arm_callback)
    r_time = rospy.Rate(1)


    for i in range(20):
        try:
            roboclaw1 = SteerClaw(0x80, "/dev/actuatorClaw", 9600, "BaseClaw")
            roboclaw1.setActuatorConstants()
        except SerialException:
            rospy.logwarn("Could not connect to Arm RoboClaw1, retrying...")
            r_time.sleep()
    rospy.loginfo("Connected to Arm RoboClaw1")

    
    for i in range(20):
        try:
            roboclaw2 = SteerClaw(0x81, "/dev/wristClaw", 9600, "WristClaw")
            roboclaw2.setPIDconstants(0.02,0.02,30)
        except SerialException:
            rospy.logwarn("Could not connect to Arm RoboClaw2, retrying...")
            r_time.sleep()
    
    rospy.loginfo("Connected to Arm RoboClaw2")


    r_time = rospy.Rate(5)
    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    roboclaw2.claw.ForwardM1(0)
    roboclaw2.claw.ForwardM2(0)

    while not rospy.is_shutdown():
        roboclaw1.update_rpm()
        roboclaw1.pub_pot(pub)
        roboclaw2.update_pid()
        roboclaw2.pub_enc(pub2)
        r_time.sleep()

    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    roboclaw2.claw.ForwardM1(0)
    roboclaw2.claw.ForwardM2(0)
