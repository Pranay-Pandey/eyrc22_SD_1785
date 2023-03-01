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
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
import rospy
import time
import numpy as np
from control.matlab import dlqr
from filterpy.kalman import KalmanFilter
from math import pi
# from cv_bridge import CvBridge

class MyDroneController_DLQR():

    def __init__(self, del_t):
        
        self.mass = 1.5 + 0.04
        self.maximum_Thrust = 30.0
        self.maximum_roll_pitch_angle = 32.3*pi/180
        self.thrust = (self.maximum_Thrust - self.mass*9.8)/500
        self.roll_pitch = self.maximum_roll_pitch_angle/500

        self.A = np.array([[1, del_t, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
						   [0, 0, 1, del_t, 0, 0],
						   [0, 0, 0, 1, 0, 0],
						   [0, 0, 0, 0, 1, del_t],
						   [0, 0, 0, 0, 0, 1]])

        self.B = np.array([[(self.roll_pitch*self.mass*9.8*(del_t**2))/2, 0, 0],
		                   [self.roll_pitch*self.mass*9.8*del_t, 0, 0],
						   [0, -(self.roll_pitch*self.mass*9.8*(del_t**2))/2, 0],
						   [0, -self.roll_pitch*self.mass*9.8*del_t, 0],
						   [0, 0, -(self.thrust*del_t**2)/2],
						   [0, 0, -self.thrust*del_t]])

        self.Q = np.array([[50, 0, 0, 0, 0, 0],
                           [0, 10, 0, 0, 0, 0],
						   [0, 0, 50, 0, 0, 0],
						   [0, 0, 0, 10, 0, 0],
						   [0, 0, 0, 0, 50, 0],
						   [0, 0, 0, 0, 0, 10]])
		
        self.R = np.array([[0.1, 0, 0],
		                   [0, 0.1, 0],
						   [0, 0, 0.1]])
        (self.K, self.S, self.E) = dlqr(self.A, self.B, self.Q, self.R)
        print("Closed Loop Gain Matrix:\n", self.K, "\n")
        print("Closed Loop Eigenvalues of the system:\n", self.E, "\n")
	
        self.kalman = KalmanFilter(dim_x=6, dim_z=3, dim_u=3)

        self.kalman.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.kalman.B = self.B
        self.kalman.F = self.A
        self.kalman.H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				                  [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
				                  [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]])
        self.kalman.P *= 10.0
        self.kalman.R *= 2.0
        self.kalman.Q *= 5.0

    def ComputeOutput(self, X):

        self.U = -self.K.dot(X)

    def AdditionalProcessing(self):

        self.Xin_0 = int(self.U[0])
        self.Xin_1 = int(self.U[1])
        self.Xin_2 = int(self.U[2])

        return self.Xin_0, self.Xin_1, self.Xin_2
    
    def State_Estimator(self, Z, U):
	    
        self.kalman.predict(U)
        self.kalman.update(Z)
        return self.kalman.x


class Edrone():
	
	def __init__(self, del_t):
		
		rospy.init_node('drone_control')	
		self.drone_position = [0.0,0.0,0.0]	
		self.setpoint = [7,-7,20]
		# self.bridge = CvBridge()


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

		self.sample_time = del_t*1000 #ms
		self.now=0.0000
		self.timechange=0.000
		self.last_time=0.0000

		self.points = [[7, -7, 20], [-7, 7, 20]]
		
		self.out_roll=0.000
		self.out_pitch=0.000
		self.out_throttle=0.000
		
		self.controller = MyDroneController_DLQR(del_t)


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.altError = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.pitchError = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.rollError = rospy.Publisher('/roll_error', Float64, queue_size=1)


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/drone/setpoint', Float32MultiArray, self.setpoint_callback)
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

	def setpoint_callback(self, msg):

		self.setpoint = [msg.data[0], msg.data[1], msg.data[2]]
		

	def Control(self):
	

		#time functions
		self.now = time.time()
		self.timechange=self.now-self.last_time
		
		#delta time must be more than step time of Gazebo, otherwise same values will be repeated 
		if ((self.timechange*1000)>self.sample_time):
			if (self.last_time!=0):			

				#Getting error of all coordinater
				Z = np.array([self.drone_position[0], self.drone_position[1], self.drone_position[2]])
				U = np.array([self.out_roll, self.out_pitch, self.out_throttle])
				estimated_states = self.controller.State_Estimator(Z, U)
				# rospy.loginfo(f"{estimated_state.shape}")
				# rospy.loginfo(f"{estimated_state}")
				self.error[0]=estimated_states[0] - self.setpoint[0]
				self.error[1]=estimated_states[2] - self.setpoint[1]
				self.error[2]=estimated_states[4] - self.setpoint[2]
				
				

				self.derr[0]=(estimated_states[1])
				self.derr[1]=(estimated_states[3])
				self.derr[2]=(estimated_states[5])

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
		if abs(max([self.drone_position[i] - self.setpoint[i] for i in range(3)], key=abs)) < 0.2 and abs(max(self.derr, key=abs)) < 0.2:
			if len(self.points) > 0:
				self.setpoint = self.points[0]
				self.points.pop(0)
		# rospy.loginfo(f"{self.points}")

		


if __name__ == '__main__':
	e_drone = Edrone(0.06)
	r = rospy.Rate(16) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.main()
		r.sleep()




###INITIAL SCRIPT FOR POSITION_HOLD
