#!/usr/bin/env python3

'''
This python file runs a ROS-node of name drone_control which holds the position of Drone on the given dummy.
This node publishes and subsribes the following topics:
		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time



class Edrone():
	
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		self.drone_position = [0.0,0.0,0.0]	# drone initial position

		self.setpoint = [] # [x_setpoint, y_setpoint, z_setpoint]
		self.waypoints = [[0,0,23],[2,0,23],[2,2,23],[-2,2,23],[-2,-2,23], [2,-2,23], [2,0,23], [0,0,23.17]]

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
		

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		# self.Kp = [0,0,328*0.04]
		# self.Ki = [0,0,0]	
		# self.Kd = [0,0,218*0.33*1000]


		#-----------------------Add other required variables for pid here ----------------------------------------------


		self.sample_time = 60
		self.prev_values = [0,0,0]
		self.max_values = 2000
		self.min_values = 1000
		self.error = [0,0,0]
		self.now=0.0000
		self.timechange=0.000
		self.errsum=[0,0,0]
		self.derr=[0,0,0]
		self.last_time=0.0000
		
		self.out_roll=0
		self.out_pitch=0
		self.out_throttle=0
	
		self.setpoint = self.waypoints[0]


		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.altError = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.pitchError = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.rollError = rospy.Publisher('/roll_error', Float64, queue_size=1)
		
	

		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		# rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		# rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		# rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)



		#------------------------------------------------------------------------------------------------------------

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



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
    
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
				#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	# def altitude_set_pid(self,alt):
	# 	self.Kp[2] = alt.Kp * 0.04# This is just for an example. You can change the ratio/fraction value accordingly
	# 	self.Ki[2] = alt.Ki * 0.009
	# 	self.Kd[2] = alt.Kd * 0.33*1000
	
		
	# #----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	# def pitch_set_pid(self,pitch):
	# 	self.Kp[0] = pitch.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
	# 	self.Ki[0] = pitch.Ki * 0.008
	# 	self.Kd[0] = pitch.Kd * 0.33*1000


	# def roll_set_pid(self,roll):
	# 	self.Kp[1] = roll.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
	# 	self.Ki[1] = roll.Ki * 0.008
	# 	self.Kd[1] = roll.Kd * 0.33*1000


	def change_target(self):
		if len(self.waypoints) == 0:
			pass
		else:
			self.setpoint = self.waypoints[0]

	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

		#time functions
		self.now = int(round(time.time() * 1000))
		self.timechange=self.now-self.last_time
		
		#delta time must be more than step time of Gazebo, otherwise same values will be repeated 
		if (self.timechange>self.sample_time):
			if (self.last_time!=0):	

				#Getting error of all coordinates		
				self.error[0]=self.drone_position[0] - self.setpoint[0]
				self.error[1]=self.drone_position[1] - self.setpoint[1]
				self.error[2]=self.drone_position[2] - self.setpoint[2]

				if len(self.waypoints) > 0 and abs(self.error[0]) < 0.2 and abs(self.error[1]) < 0.2 and abs(self.error[2]) < 0.2:
					self.waypoints.pop(0)
					self.change_target()

				if abs(self.error[2]) <= 0.2:
					throttle_Kp = 66*0.04
					throttle_Kd = 218*0.33*1000
					throttle_Ki = 0
				elif abs(self.error[2]) <= 2.0:
					throttle_Kp = 328*0.04
					throttle_Kd = 218*0.33*1000
					throttle_Ki = 0
				else:
					throttle_Kp = 328*0.04
					throttle_Kd = 0
					throttle_Ki = 0

				if abs(self.error[0]) <= 0.1:
					roll_Kp = 66*0.04
					roll_Kd = 218*0.33*1000
					roll_Ki = 0
				elif abs(self.error[0]) <= 1.0:
					roll_Kp = 328*0.04
					roll_Kd = 218*0.33*1000
					roll_Ki = 0
				else:
					roll_Kp = 328*0.04
					roll_Kd = 0
					roll_Ki = 0

				if abs(self.error[0]) <= 0.1:
					pitch_Kp = 328*0.04
					pitch_Kd = 218*0.33*1000
					pitch_Ki = 0
				elif abs(self.error[0]) <= 1.0:
					pitch_Kp = 328*0.04
					pitch_Kd = 218*0.33*1000
					pitch_Ki = 0
				else:
					pitch_Kp = 328*0.04
					pitch_Kd = 0
					pitch_Ki = 0

				

				#Integration for Ki
				self.errsum[0]=self.errsum[0]+(self.error[0]*self.timechange)
				self.errsum[1]=self.errsum[1]+(self.error[1]*self.timechange)
				self.errsum[2]=self.errsum[2]+(self.error[2]*self.timechange)


				#Derivation for Kd
				self.derr[0]=(self.error[0]-self.prev_values[0])/self.timechange
				self.derr[1]=(self.error[1]-self.prev_values[1])/self.timechange
				self.derr[2]=(self.error[2]-self.prev_values[2])/self.timechange


				self.out_roll = int((roll_Kp*self.error[0])+(roll_Kd*self.derr[0])-(self.errsum[0]*roll_Ki))
				self.out_pitch = int((pitch_Kp*self.error[1])+(pitch_Kd*self.derr[1])-(self.errsum[1]*pitch_Ki))
				self.out_throttle = int((throttle_Kp*self.error[2])+(throttle_Kd*self.derr[2])-(self.errsum[2]*throttle_Ki))

				#Calculating output in 1500
				self.cmd.rcRoll=1500 - self.out_roll
				self.cmd.rcPitch=1500 + self.out_pitch
				self.cmd.rcThrottle=1500 + self.out_throttle

				
				#Checking min and max threshold and updating on true
				#Throttle Conditions
				self.cmd.rcThrottle = min(self.max_values,max(self.min_values,self.cmd.rcThrottle))
				self.cmd.rcPitch = min(self.max_values,max(self.min_values,self.cmd.rcPitch))
				self.cmd.rcRoll = min(self.max_values,max(self.min_values,self.cmd.rcRoll))




				#Publishing values on topic 'drone command'
				self.command_pub.publish(self.cmd)

				
				#Updating prev values for all axis
				self.prev_values[0]=self.error[0]
				self.prev_values[1]=self.error[1]
				self.prev_values[2]=self.error[2]

				# self.cmd.rcRoll = self.cmd.rcRoll if self.error[0] >= 0.15 else 1500
				# self.cmd.rcPitch = self.cmd.rcPitch if self.error[1] >= 0.15 else 1500
				# self.cmd.rcThrottle = self.cmd.rcThrottle if self.error[2] >= 0.25 else 1500
		 		
		 	#Updating last time value	
			self.last_time = self.now
	

			#Getting values for Plotjuggler
			self.rollError.publish(self.error[0])
			self.pitchError.publish(self.error[1])
			self.altError.publish(self.error[2])


		 
		
		
	#------------------------------------------------------------------------------------------------------------------------



if __name__ == '__main__':
	e_drone = Edrone()
	r = rospy.Rate(10) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()




###INITIAL SCRIPT FOR POSITION_HOLD