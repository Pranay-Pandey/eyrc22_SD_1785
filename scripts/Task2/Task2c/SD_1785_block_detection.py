#!/usr/bin/env python3

'''
This python file runs a ROS-node of name drone_control which holds the position of Drone on the given dummy.
This node publishes and subsribes the following topics:
		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				
		/pitch_error			
		/roll_error				
					
								
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import rospy
import time
from cv_bridge import CvBridge
import numpy as np
from control.matlab import lqr
from math import cos, pi
import cv2

class MyDroneController():

    def __init__(self) -> None:
        
        self.mass = 1.5 + 0.04
        self.thrust = 15/500
        self.f_thrust = 15/500
        self.roll_pitch_factor = ((4.5*pi)/180)/85

        self.A = np.array([[0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 1, 0, 0],
						   [0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 0, 0, 1],
						   [0, 0, 0, 0, 0, 0]])

        self.B = np.array([[0, 0, 0],
		                   [self.f_thrust, 0, 0],
						   [0, 0, 0],
						   [0, self.f_thrust, 0],
						   [0, 0, 0],
						   [0, 0, self.thrust]])

        self.Q = np.array([[500, 0, 0, 0, 0, 0],
                           [0, 500, 0, 0, 0, 0],
						   [0, 0, 500, 0, 0, 0],
						   [0, 0, 0, 500, 0, 0],
						   [0, 0, 0, 0, 200, 0],
						   [0, 0, 0, 0, 0, 200]])
		
        self.R = np.array([[0.1, 0, 0],
		                   [0, 0.1, 0],
						   [0, 0, 0.1]])

        (self.K, self.S, self.E) = lqr(self.A, self.B, self.Q, self.R)

    def ComputeOutput(self, X):

        self.U = self.K.dot(X)

    def AdditionalProcessing(self):

        self.Xin_0 = -int(self.U[0])
        self.Xin_1 = int(self.U[1])
        self.Xin_2 = int(self.U[2]/(cos(self.Xin_0*self.roll_pitch_factor)*cos(self.Xin_1*self.roll_pitch_factor) + 1e-7))

        return self.Xin_0, self.Xin_1, self.Xin_2


class Edrone():
	
	def __init__(self):
		
		rospy.init_node('drone_control')	
		self.drone_position = [0.0,0.0,0.0]	
		self.setpoint = [0,0,20]
		self.bridge = CvBridge()


		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		



		#-----------------------Add other required variables for pid here ----------------------------------------------


		self.error = [0,0,0]
		self.prev_values = [0,0,0]
		self.derr=[0,0,0]

		self.throttle_min_values = -500
		self.throttle_max_values = 500

		self.roll_pitch_min_values = -40
		self.roll_pitch_max_values = 40

		self.sample_time = 60
		self.now=0.0000
		self.timechange=0.000
		self.last_time=0.0000
		
		self.out_roll=0.000
		self.out_pitch=0.000
		self.out_throttle=0.000

		self.flag = 0
		self.flag2 = 0

		
		self.controller = MyDroneController()


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.altError = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.pitchError = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.rollError = rospy.Publisher('/roll_error', Float64, queue_size=1)


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber("/edrone/camera_rgb/image_raw", Image, self.ImageCallback)
		self.arm() # ARMING THE DRONE



	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)


	def whycon_callback(self,msg):
		
		self.drone_position[0] = msg.poses[0].position.x
				#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#---------------------------------------------------------------------------------------------------------------

	def ImageCallback(self, msg):

		self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		
	def ImageProcessing(self):

		if hasattr(self, "img"):
			img_hsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV) # converting to HSV colour space for colour detection

			lower = np.array([0,140,87]) # getting array of lower bounds values
			upper = np.array([57,255,255]) # getting array of upper bounds values

			mask = cv2.inRange(img_hsv,lower,upper) # masking the yellow box region

			cont, hier = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE) # finding the contours of the yellow block

			if len(cont) > 0:
				yellow_block_contour = max(cont, key=cv2.contourArea) # just a safe check

				# calculating center of area using area moments
				M = cv2.moments(yellow_block_contour)
				if M["m00"] != 0:
					cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
					cv2.circle(self.img, (cX, cY), 5, (0, 0, 255), -1)
					h,w = img_hsv.shape[:2]
					cv2.circle(self.img, (w//2, h//2), 5, (255, 0, 0), -1)
					if self.flag == 0:
						X = cX - w//2
						Y = cY - h//2
						self.err_x = X*0.001
						self.err_y = Y*0.001
						self.setpoint[2] = 23
						self.flag2 = 1
			else:
				self.flag2 = 0
				self.setpoint[2] = 20

			# cv2.imshow("image",self.img)
			# cv2.waitKey(1)


	def Control(self):
	

		#time functions
		self.now = time.time()
		self.timechange=self.now-self.last_time
		
		#delta time must be more than step time of Gazebo, otherwise same values will be repeated 
		if ((self.timechange*1000)>self.sample_time):
			if (self.last_time!=0):			

				#Getting error of all coordinates		
				self.error[0]=self.drone_position[0] - self.setpoint[0]
				self.error[1]=self.drone_position[1] - self.setpoint[1]
				self.error[2]=self.drone_position[2] - self.setpoint[2]

				if self.flag2 == 1:
					self.error[0] = -self.err_x
					self.error[1] = -self.err_y

				self.derr[0]=(self.error[0]-self.prev_values[0])/self.timechange
				self.derr[1]=(self.error[1]-self.prev_values[1])/self.timechange
				self.derr[2]=(self.error[2]-self.prev_values[2])/self.timechange

				X = np.array([[self.error[0]],
                              [self.derr[0]],
							  [self.error[1]],
							  [self.derr[1]],
							  [self.error[2]],
							  [self.derr[2]]])

				self.controller.ComputeOutput(X)
				self.out_roll, self.out_pitch, self.out_throttle = self.controller.AdditionalProcessing()
		
				#Checking min and max threshold and updating on true
				self.out_roll = min(self.roll_pitch_max_values,max(self.roll_pitch_min_values,self.out_roll))
				self.out_pitch = min(self.roll_pitch_max_values,max(self.roll_pitch_min_values,self.out_pitch))
				self.out_throttle = min(self.throttle_max_values,max(self.throttle_min_values,self.out_throttle))

				#Calculating output in 1500
				self.cmd.rcRoll=1500 + self.out_roll
				self.cmd.rcPitch=1500 + self.out_pitch
				self.cmd.rcThrottle=1500 + self.out_throttle

				#Updating prev values for all axis
				self.prev_values[0]=self.error[0]
				self.prev_values[1]=self.error[1]
				self.prev_values[2]=self.error[2]
		 		
				#Publishing values on topic 'drone command'
				self.command_pub.publish(self.cmd)
		 	#Updating last time value	
			self.last_time = self.now
	

			#Getting values for Plotjuggler
			self.rollError.publish(self.error[0])
			self.pitchError.publish(self.error[1])
			self.altError.publish(self.error[2])
		
	#------------------------------------------------------------------------------------------------------------------------

	def main(self):

		self.Control()
		self.ImageProcessing()
		# rospy.loginfo("274")
		if len(self.points) > 0 and self.flag == 0:
			# rospy.loginfo("276")
			self.flag = 1
			# rospy.loginfo(f"{self.setpoint}")
			self.setpoint = self.points[0]
		if self.flag == 1 and abs(max([self.drone_position[i] - self.setpoint[i] for i in range(3)], key=abs)) < 0.2 and abs(max(self.derr, key=abs)) < 0.2:
			# rospy.loginfo("283")
			# rospy.loginfo(f"{self.setpoint}")
			self.flag = 0
			self.points.pop(0)

		


if __name__ == '__main__':
	e_drone = Edrone()
	e_drone.points = [[0,8.5,20],[-8.5, 8.5, 20]]
	e_drone.flag = 0
	r = rospy.Rate(16) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.main()
		r.sleep()




###INITIAL SCRIPT FOR POSITION_HOLD