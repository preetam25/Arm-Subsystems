import sys
import signal
import numpy as np
import pdb
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Arc
from matplotlib.lines import Line2D

#SIGINT handler
def sigint_handler(signal, frame):
    #Do something while breaking
    pdb.set_trace()
    print("Exiting")
    sys.exit(0)
#---------------------------------------------------------------------------
class Arm_Simulator:

	L1=0.52
	L2=0.37
	L3=0.35
	alpha_min =(np.pi/180)*30
	alpha_max =(np.pi/180)*70
	beta_min =(np.pi/180)*10
	beta_max =(np.pi/180)*40
	gamma_min =-(np.pi/180)*50
	gamma_max =-(np.pi/180)*10
	
	def __init__(self, current_alpha = 0.698132, current_beta = 0.349066, current_gamma = -0.698132, err = 0.01):
		self.alpha = current_alpha
		self.beta = current_beta
		self.gamma = current_gamma
		self.error = err

	def calc_target_angles(self, target_x, target_y):
		self.current_x = self.L1*np.cos(self.alpha)+self.L2*np.cos(self.beta)+self.L3*np.cos(self.gamma)
		self.current_y = self.L1*np.sin(self.alpha)+self.L2*np.sin(self.beta)+self.L3*np.sin(self.gamma)
		self.current_r = np.sqrt(self.current_x**2+self.current_y**2)
		
		target_r = np.sqrt(target_x**2+target_y**2)
		
		if target_r > self.current_r:
			while np.abs(target_r-self.current_r) > self.error:
				if self.beta < self.beta_max: self.beta = self.beta + 0.001
				elif self.gamma < self.gamma_max: self.gamma = self.gamma + 0.001
				else: 
					print("Not possible, move the rover forward")
					break
				self.current_x = self.L1*np.cos(self.alpha)+self.L2*np.cos(self.beta)+self.L3*np.cos(self.gamma)
				self.current_y = self.L1*np.sin(self.alpha)+self.L2*np.sin(self.beta)+self.L3*np.sin(self.gamma)
				self.current_r = np.sqrt(self.current_x**2+self.current_y**2)
		elif target_r < self.current_r:
			while np.abs(target_r-self.current_r) > self.error:
				if self.beta > self.beta_min: self.beta = self.beta - 0.001
				elif self.gamma > self.gamma_min: self.gamma = self.gamma - 0.001
				else: 
					print("Not possible, move the rover backward")
					break
				self.current_x = self.L1*np.cos(self.alpha)+self.L2*np.cos(self.beta)+self.L3*np.cos(self.gamma)
				self.current_y = self.L1*np.sin(self.alpha)+self.L2*np.sin(self.beta)+self.L3*np.sin(self.gamma)
				self.current_r = np.sqrt(self.current_x**2+self.current_y**2)

		target_theta = np.arctan(target_y/target_x)
		self.current_theta = np.arctan(self.current_y/self.current_x)

		if target_theta > self.current_theta: 
			while np.abs(target_theta - self.current_theta) > self.error:
				if self.alpha < self.alpha_max: self.alpha = self.alpha + 0.001
				else: 
					print("Not possible, move the rover backward")
					break
				self.current_x = self.L1*np.cos(self.alpha)+self.L2*np.cos(self.beta)+self.L3*np.cos(self.gamma)
				self.current_y = self.L1*np.sin(self.alpha)+self.L2*np.sin(self.beta)+self.L3*np.sin(self.gamma)
				self.current_theta = np.arctan(self.current_y/self.current_x)
		elif target_theta < self.current_theta: 
			while np.abs(target_theta - self.current_theta) > self.error:
				if self.alpha > self.alpha_min: self.alpha = self.alpha + 0.001
				else: 
					print("Not possible, move the rover forward")
					break
				self.current_x = self.L1*np.cos(self.alpha)+self.L2*np.cos(self.beta)+self.L3*np.cos(self.gamma)
				self.current_y = self.L1*np.sin(self.alpha)+self.L2*np.sin(self.beta)+self.L3*np.sin(self.gamma)
				self.current_theta = np.arctan(self.current_y/self.current_x)
		
		return round((180/np.pi)*self.alpha,1), round((180/np.pi)*self.beta,1), round((180/np.pi)*self.gamma,1)	


	def plot_arm(self,ax,plot_sil=False, col='blue', col2='cyan', phi_lx=-0.1, phi_ly=0.1, scat_param='ro'):
	    joint1_x=self.L1*np.cos(self.alpha)
	    joint1_y=self.L1*np.sin(self.alpha)
	    joint2_x=joint1_x+self.L2*np.cos(self.beta)
	    joint2_y=joint1_y+self.L2*np.sin(self.beta)
	    target_x=joint2_x+self.L3*np.cos(self.gamma)
	    target_y=joint2_y+self.L3*np.sin(self.gamma)

	    if(plot_sil==False):	    
	        limb1=Line2D([0,joint1_x],[0,joint1_y],linewidth=2, linestyle = "-", color=col)
	        limb2=Line2D([joint1_x,joint2_x],[joint1_y,joint2_y],linewidth=2, linestyle = "-", color=col)    
	        limb3=Line2D([joint2_x,target_x],[joint2_y,target_y],linewidth=2, linestyle = "-", color=col)
	        ax.add_line(limb1)
	        ax.add_line(limb2)
	        ax.add_line(limb3)
	        plt.plot([0,joint1_x,joint2_x,target_x],[0,joint1_y,joint2_y,target_y],scat_param)	        
	    # return (target_x,target_y)

signal.signal(signal.SIGINT, sigint_handler)

plt.rcParams["figure.figsize"] = (12,12)
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.set_xlim(-0.5,2)
ax.set_ylim(-0.5,2)

Arm = Arm_Simulator((np.pi/180)*60, (np.pi/180)*30, -(np.pi/180)*10)
Arm.plot_arm(ax,col='red')
print(Arm.calc_target_angles(0.9,0.4))
Arm.plot_arm(ax,col='green')
plt.scatter(0.9,0.4,s=100)
plt.legend()
plt.show()