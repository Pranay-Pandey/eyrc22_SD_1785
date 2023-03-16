#!/usr/bin/env python3

"""
* Team Id : 1785
* Author List : Harshit Sharma, Pranay Pandey, Aniket Kumar Roy, Dhruv Darshan Shah
* Filename: SD_1234_task6.py
* Theme: Sentinel Drone (SD 2022)
* Functions: Change_setpoint, pixel2coord, g_ref_info, Image_processing, make_list, Image_Callback, pid, publish_data_to_rpi, shutdown_hook
             arm, disarm, main, Wait, Wait_Once
* Global Variables: MIN_THROTTLE, BASE_THROTTLE, MAX_THROTTLE, SUM_ERROR_THROTTLE_LIMIT, MIN_ROLL, BASE_ROLL, MAX_ROLL, SUM_ERROR_ROLL_LIMIT
                    MIN_PITCH = 1400, BASE_PITCH, MAX_PITCH, SUM_ERROR_PITCH_LIMIT, Height, flag1, count, num, PID_OUTPUT_VALUES
"""


# standard imports
import copy
import time
import math as m
# third-party imports
import scipy.signal
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from pid_tune.msg import PidTune
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import csv
from osgeo import gdal
from sentinel_drone.msg import Geolocation

from sentinel_drone_driver.msg import PIDError, RCMessage
from sentinel_drone_driver.srv import CommandBool, CommandBoolResponse


# Minimum, Base and Maximum throttle values to be feed
MIN_THROTTLE = 1250
BASE_THROTTLE = 1300
MAX_THROTTLE = 1400

SUM_ERROR_THROTTLE_LIMIT = 6000 # Integral anti windup sum value


# Minimum, Base and Maximum roll values to be feed
MIN_ROLL = 1400
BASE_ROLL = 1510
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 2000 # Integral anti windup sum value

# Minimum, Base and Maximum pitch values to be feed
MIN_PITCH = 1400
BASE_PITCH = 1500
MAX_PITCH = 1600
SUM_ERROR_PITCH_LIMIT = 2000 # Integral anti windup sum value

Height = 23 # height of the drone z-coordinate in whycon frame at which the drone will fly
count=0 # no. of frames the yellow block is detected in image. the drone will not immediately capture the frame, 
#         it will wait for the yellow block to be detected for a certain no. of frames then it will capture it

num=1 #The variable accounting for the number of images taken of the yellow boxes in the arena, this will update during runtime

# This will be used to store data for filtering purpose
PID_OUTPUT_VALUES = [[], [], []]

###########################################################

#This class is used to generate delays in runtime

class Delay():

    ##############################################################

    #Function Name:constructor of this class
    #Input: delay -> input time in seconds for which the control flow of the program will be paused
    #Output:None
    #Logic: to pause the control flow 
    #Example call:Delay(5)

    def __init__(self, delay):

        self.now = 0 # for noting current time
        self.Always_on = False # A flag
        self.initial_time = time.time() # noting initial time
        self.delay = delay # delay in seconds
    
    ##############################################################

    #Function Name:Wait
    #Input: none
    #Output: bool value
    #Logic: if the waiting time is completed it returns true and reset the time 
    #Example call:Wait()

    def Wait(self): # This function will delay the program execution by @delay seconds everytime it is called

        self.now = time.time() # noting current time
        if (self.now - self.initial_time) >= self.delay: # comparing with initial time for delay
            self.initial_time = self.now # resetting initial time
            return True
        
    ##############################################################

    #Function Name:Wait_Once
    #Input: none
    #Output: bool value
    #Logic: if the waiting time is completed it returns true but it does not reset the time, it delay the program only once
    #Example call:Wait_Once()
    
    def Wait_Once(self): # this function will delay program execution only Once

        if self.Always_on == False: # if it has not delayed any program execution before
            self.now = time.time() # noting current time
            if (self.now - self.initial_time) >= self.delay: # comparing with initial time for delay
                self.Always_on = True # now it will always be open i.e. it will now never cause program execution delay
                return True
        else: # if it has previously delayed a program execution
            return True # no need to delay anymore on later subsequent function calls
        

########################################################################################################################

#This class is the controller of the drone

class DroneController:

    ##############################################################

    #Function Name: constructor
    #Input: none
    #Output: none
    #Logic: initialising control parameters
    #Example call:DroneController()
    
    def __init__(self):

        self.rc_message = RCMessage() #it store the roll,pitch and yaw command
        self.rc_message_filtered = RCMessage() #it stores the filtered roll,pitch and yaw command to be sent
        self.drone_whycon_pose_array = PoseArray() #it captures the drone position from whycon node
        self.last_whycon_pose_received_at = None #it records the last time of the whycon captured drone position
        self.is_flying = False #this variable stores whether the drone is flying or not

        self.csv_loc = r'/home/aniket/images/data.csv' #location of the csv file in local storage where the coordinates will be stored after georeferencing

        self.delay = Delay(5) #calling Delay and storing in variable delay
        
        self.bridge = CvBridge() #conversion from rosmsg to cv image


        self.lower = np.array([20, 100, 163]) #lower bound of yellow color for detection
        self.upper = np.array([39, 189, 232]) #upper bound of yellow color for detection

        # Setpoints for x, y, z respectively
        self.set_points = [0, 0, Height] #initial set point of the drone
        self.make_list() #calling this function which is defined down, this creates the waypoint list for drone's traversal in the entire arena

        self.error = [0, 0, 0]   # Error for roll, pitch and throttle

        # Creates variables for previous error and sum_error
        self.previous_error = [0, 0, 0]
        self.sum_error = [0, 0, 0]
        self.derr = [0, 0, 0]

        # PID gains for roll, pitch and throttle with multipliers
        self.Kp = [20,  -20,  13.54]

        # Create variables for Kd and Ki similarly
        self.Kd = [800, -800, 254.4]
        self.Ki = [0.0001*275, -0, 0.0263]

        # Initialize rosnode
        node_name = "controller"
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown_hook)

        # Create subscriber for WhyCon

        rospy.Subscriber("/whycon/poses", PoseArray,
                         self.whycon_poses_callback)

        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required

        rospy.Subscriber(
            "/video_frames", Image, self.Image_Callback
        )

        # Create publisher for sending commands to drone

        self.rc_pub = rospy.Publisher(
            "/sentinel_drone/rc_command", RCMessage, queue_size=1
        )

        # Create publisher for publishing errors for plotting in plotjuggler

        self.pid_error_pub = rospy.Publisher(
            "/sentinel_drone/pid_error", PIDError, queue_size=1
        )

        # publishers for sum_errors
        self.sum_error_publisherx = rospy.Publisher(
            "/sentinel_drone/sum_errorx", Float64, queue_size=1)
        self.sum_error_publishery = rospy.Publisher(
            "/sentinel_drone/sum_errory", Float64, queue_size=1)
        self.sum_error_publisherz = rospy.Publisher(
            "/sentinel_drone/sum_errorz", Float64, queue_size=1)
        # publishers for error limits
        self.errll = rospy.Publisher(
            "/sentinel_drone/lowerlimit", Float64, queue_size=1)
        self.errul = rospy.Publisher(
            "/sentinel_drone/upperlimit", Float64, queue_size=1)
        self.der = rospy.Publisher(
            "/sentinel_drone/der", Float64, queue_size=1)
        
        self.latlong = rospy.Publisher('/geolocation', Geolocation, queue_size=3) #creating latlong variable for publishing on rostopic /geolocation 

        #initialising object named mssssg of Geolocation and giving its initial values according to the rosmsg
        self.mssssg = Geolocation()
        self.mssssg.objectid = str(0)
        self.mssssg.lat = np.float32(0)
        self.mssssg.long = np.float32(0)


    ##############################################################

    #Function Name:Change_setpoint
    #Input: none
    #Output: none
    #Logic: it changes the setpoint of the drone if the previous setpoint is reached and also lands the drone if all setpoints are traversed.
    #Example call:Change_setpoint()

    def Change_setpoint(self):

        if max(self.error, key=abs) <= 2.0 and max(self.derr, key=abs) <= 1.0: #if the drone position and velocity are stable
            if self.delay.Wait(): #waits here for some time as defined in variable delay
                if len(self.target_points) > 0: #if some setpoint is remaining to be traversed then enters
                    self.set_points = self.target_points[0] #changes the setpoint variable as the first element in the target_points array
                    print(self.set_points)
                    self.target_points.pop(0) #pops the first element of the waypoint array
                elif len(self.target_points) == 0: #if all setpoints are traversed, then disarm the drone
                    self.disarm()
                    self.main() #here the main function is called which is defined below in the class and this will georeference all the images captured and stored locally

    ##############################################################

    #Function Name:pixel2coord
    #Input: x, y , a, b, xoff , d, e , yoff
    #Output: returns tuple
    #Logic: returns global coordinates from pixel x,y coords
    #Example call:pixel2coord(x, y , a, b, xoff , d, e , yoff)

    def pixel2coord(self, x, y , a, b, xoff , d, e , yoff):
            """Returns global coordinates from pixel x, y coords"""
            xp = a * x + b * y + xoff
            yp = d * x + e * y + yoff
            return(xp, yp)
    
    ##############################################################

    #Function Name:g_ref_info
    #Input: num->the index of the photo which is being called for the manipulation
    #Output: return type void but saves the georeferenced images in the same local directory in which the captured images are stored
    #Logic: reads the image using the parameter index of the yellow box photo, feature matches and georeferences the images.
    #Example call:g_ref_info(1)

    def g_ref_info(self,num):

        src = "/home/aniket/images/photo"+str(num)+".png" #path of the saved image
        img1 = cv2.imread(src) #reading the image 
        img2 = cv2.imread('/home/aniket/images/task2d.tif') #reading the satellite image

        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) #converting to gray scale
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        sift = cv2.xfeatures2d.SIFT_create() #calling the feature matching function in openCV for SIFT 

        kp1, descriptors_1 = sift.detectAndCompute(img1,None) #extracting the keypoints and descriptors from each image
        kp2, descriptors_2 = sift.detectAndCompute(img2,None)

        bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True) #initialising the matcher

        matches = bf.match(descriptors_1,descriptors_2) #performing matches for both the images

        matches = sorted(matches, key = lambda x:x.distance) #sorting the best matches

        l1 = [] #creating empty lists for use
        l2 = []

        ds = gdal.Open('/home/aniket/images/task2d.tif') #using the satellite image
        xoff, a, b, yoff, d, e = ds.GetGeoTransform() #getting the parameters for calling the pixel2coord function 

        for mat in matches[:40]: #for the first 40 best matches append the pixel coordinates into the lists l1 and l2 before and after pixel2coord function on each pixel

            img1_idx = mat.queryIdx 
            img2_idx = mat.trainIdx

            #x - columns
            #y - rows
            #Get the coordinates
            (x1, y1) = kp1[img1_idx].pt
            (x2, y2) = kp2[img2_idx].pt
            l1.append((x1,y1))
            l2.append(self.pixel2coord(x2,y2 , a , b, xoff, d , e, yoff))

        #changing the coordinate system and running the CLI command from python using subprocess module

        from_SRS = "EPSG:4326"
        
        to_SRS = "EPSG:4326"
        
        src='/home/aniket/images/task2d.tif'
        
        dest= '/home/aniket/images/updated_task2d'+str(num)+'.tif'
        
        cmd_list = ["gdalwarp","-r", "near", "-order", "3", "-s_srs", from_SRS, "-t_srs", to_SRS, "-overwrite", src, dest]
        
        subprocess.run(cmd_list)

        #using the saved image and to see the progress in the terminal
        
        src="/home/aniket/images/photo"+str(num)+".png"
        dest="/home/aniket/images/updated_task2d"+str(num)+".tif"
        cmd_list = ["gdal_translate"]
        for i in range(len(l1)):
            
            cmd_list.append("-gcp")
            cmd_list.append(str(l1[i][0]))
            cmd_list.append(str(l1[i][1])) 
            cmd_list.append(str(l2[i][0]))
            cmd_list.append(str(l2[i][1]))

        cmd_list.append("-of")
        cmd_list.append("GTiff")
        cmd_list.append(src)
        cmd_list.append(dest)
        subprocess.run(cmd_list)

        #georeferencing and changing the coordinate system of the images

        from_SRS = "EPSG:4326"
        
        to_SRS = "EPSG:4326"
        
        src='/home/aniket/images/updated_task2d'+str(num)+'.tif'
        
        dest= '/home/aniket/images/fupdated_task2d'+str(num)+'.tif'
        
        cmd_list = ["gdalwarp","-r", "near", "-order", "3", "-s_srs", from_SRS, "-t_srs", to_SRS, "-overwrite", src, dest]
        
        subprocess.run(cmd_list)

    ##############################################################

    #Function Name:Image_processing
    #Input: img -> the image of the drone camera onboard
    #Output: saves the images of the detected yellow boxes in the arena in local storage given path
    #Logic: using color segmentation and contour detection for yellow boxes in the image
    #Example call:Image_processing(img)

    def Image_processing(self, img):

        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) #converting to hsv image
        mask = cv2.inRange(hsv,self.lower,self.upper) #creates mask with the defined ranges for the yellow color

        cont, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #contour extraction

        if cont: #if contour is detected
            cnt = max(cont, key=cv2.contourArea) #extracting the contour with maximum area
            global num,count
            if cv2.contourArea(cnt) >= 100: #filtering the small contours having area less than 100 square pixels
                M = cv2.moments(cnt)
                cX, cY = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]) #extracting the COM of the contour
                print("detected block: ",cX,cY) #prints the pixel 
                count+=1 #incrementing the count variable for the number of frames detected in which the yellow box is detected
                if(count>=50): #if the number of frames detected is more than 50 in which yellow box is detected
                    src = "/home/aniket/images/photo"+str(num)+".png" #path of the image to be saved
                    ker = np.array([[0,-1,0],
                                    [-1,5,-1],
                                    [0,-1,0]])
                    sharp_img = cv2.filter2D(src=img,ddepth=-1,kernel=ker) #sharpening the image using the defined kernel
                    cv2.imwrite(src,sharp_img)
                    num+=1 #incrementing the number of images saved variable
                    count=0 #redefining the count to zero
                    if len(self.target_points) > 0: #change the setpoint since the image has been saved near that particular location
                        self.set_points = self.target_points[0]
                        self.target_points.pop(0)
    
     ##############################################################

    #Function Name:make_list
    #Input: none
    #Output: saves the setpoints for waypoint mission of drone in target_points
    #Logic: linear traversal from one column start to next column end and vice versa after traversing all the rows or considering x and y which is evident in the video
    #Example call:make_list()
                            
    def make_list(self):
        self.x_init = -6.1 #first setpoint x coordi
        self.y_init = 4.6 #first setpoint y coordi
        self.x_diff = 3.0 #diff bw subsequent x coordi
        self.y_diff = 4.0 #diff bw subsequent y coordi
        self.x_init -= self.x_diff #to make the first setpoint as x_init, y_init
        
        self.target_points = [] #initialising the list 
        self.no_of_points = 4 #defining n for nxn grid
        for i in range(self.no_of_points): #making the grid
            self.x_init = self.x_init + self.x_diff
            self.y_init += m.pow(-1,i)*self.y_diff
            for j in range(self.no_of_points):
                self.y_init = self.y_init + m.pow(-1,i+1)*self.y_diff
                self.target_points.append([self.x_init,self.y_init,Height])
        for i in range(23, 28): #making the setpoint list for landing 
            self.target_points.append([0,0,i])

        print(self.target_points)     
    
    ##############################################################

    #Function Name:Image_Callback
    #Input: msg->rosmsg from drone onboard camera rostopic /video_frames
    #Output: displays onscreen the image being taken by the drone camera
    #Logic: calls back
    #Example call:Image_Callback(msg)

    def Image_Callback(self, msg):

        self.img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.Image_processing(self.img)
        cv2.imshow("img",self.img)
        cv2.waitKey(1)

    ##############################################################

    #Function Name:whycon_poses_callback
    #Input: img -> the image of the drone camera onboard
    #Output: updates the position of the drone since tracks the coordinate of the whycon marker
    #Logic: calls back
    #Example call:whycon_poses_callback(msg)

    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = rospy.get_rostime().secs
        self.drone_whycon_pose_array = msg

    ##############################################################

    #Function Name:pid
    #Input: none
    #Output: output the roll,pitch and throttle values for the drone
    #Logic: PID Controller algorithm
    #Example call:pid()
    

    def pid(self):          # PID algorithm

        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        self.error[0] = self.drone_whycon_pose_array.poses[0].position.x - \
            self.set_points[0]
        roll_derivative_error = self.error[0]-self.previous_error[0]
        self.previous_error[0] = self.error[0]
        roll_sum_error = self.sum_error[0]
        self.sum_error[0] += self.error[0]

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis
        self.error[1] = self.drone_whycon_pose_array.poses[0].position.y - \
            self.set_points[1]
        # self.error[1] *= -1
        pitch_derivative_error = self.error[1]-self.previous_error[1]
        # pitch_derivative_error *= -1
        self.previous_error[1] = self.error[1]
        pitch_sum_error = self.sum_error[1]
        self.sum_error[1] += self.error[1]

        # 2 : calculating Error, Derivative, Integral for Throttle error : z axis
        self.error[2] = self.drone_whycon_pose_array.poses[0].position.z - \
            self.set_points[2]
        throttle_derivative_error = self.error[2]-self.previous_error[2]
        throttle_derivative_error = round(throttle_derivative_error, 1)
        self.previous_error[2] = self.error[2]
        throttle_sum_error = self.sum_error[2]
        self.sum_error[2] += self.error[2]

        self.derr = [roll_derivative_error, pitch_derivative_error, throttle_derivative_error]

        # Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)

        if self.sum_error[0] > SUM_ERROR_ROLL_LIMIT:
            self.sum_error[0] = SUM_ERROR_ROLL_LIMIT
        if self.sum_error[0] < -SUM_ERROR_ROLL_LIMIT:
            self.sum_error[0] = -SUM_ERROR_ROLL_LIMIT

        if self.sum_error[1] > SUM_ERROR_PITCH_LIMIT:
            self.sum_error[1] = SUM_ERROR_PITCH_LIMIT
        if self.sum_error[1] < -SUM_ERROR_PITCH_LIMIT:
            self.sum_error[1] = -SUM_ERROR_PITCH_LIMIT

        if self.sum_error[2] > SUM_ERROR_THROTTLE_LIMIT:
            self.sum_error[2] = SUM_ERROR_THROTTLE_LIMIT
        if self.sum_error[2] < -SUM_ERROR_THROTTLE_LIMIT:
            self.sum_error[2] = -SUM_ERROR_THROTTLE_LIMIT

        # Write the PID equations and calculate the self.rc_message.rc_throttle, self.rc_message.rc_roll, self.rc_message.rc_pitch
        self.rc_message.rc_roll = BASE_ROLL + \
            self.error[0]*self.Kp[0]+roll_derivative_error * \
            self.Kd[0]+roll_sum_error*self.Ki[0]

        self.rc_message.rc_pitch = BASE_PITCH + \
            self.error[1]*self.Kp[1]+pitch_derivative_error * \
            self.Kd[1]+pitch_sum_error*self.Ki[1]

        self.rc_message.rc_throttle = BASE_THROTTLE + \
            self.error[2]*self.Kp[2]+throttle_derivative_error * \
            self.Kd[2]+throttle_sum_error*self.Ki[2]

        # Send constant 1500 to rc_message.rc_yaw

        self.rc_message.rc_yaw = np.uint16(1500)

    # ------------------------------------------------------------------------------------------------------------------------

        # publishing alt error, roll error, pitch error, drone message

        # self.rc_pub.publish(self.rc_message)
        self.der.publish(throttle_derivative_error*self.Kd[2])
        self.publish_data_to_rpi(
            self.rc_message.rc_roll, self.rc_message.rc_pitch, self.rc_message.rc_throttle)
        # Publish error messages for plotjuggler debugging

        self.pid_error_pub.publish(
            PIDError(
                roll_error=self.error[0],
                pitch_error=self.error[1],
                throttle_error=self.error[2],
                zero_error=0
            )
        )

        #change the setpoint once at that point delay of defined seconds is over
        self.Change_setpoint()

    ##############################################################

    #Function Name:publish_data_to_rpi
    #Input: roll, pitch, throttle -> values of the roll,pitch and throttle coming from the output of PID algorithm function
    #Output: filters these values removing the noises
    #Logic: uses the butterworth filter for filtering the roll,pitch and throttle values
    #Example call:publish_data_to_rpi(roll, pitch, throttle)

    def publish_data_to_rpi(self, roll, pitch, throttle):

        # self.rc_message.rc_throttle = np.uint16(throttle)

        self.rc_message.rc_yaw = np.uint16(1500)

        # NOTE: There is noise in the WhyCon feedback and the noise gets amplified because of derivative term, this noise is multiplied by high Kd gain values and create spikes in the output.
        #       Sending data with spikes to the drone makes the motors hot and drone vibrates a lot. To reduce the spikes in output, it is advised to pass the output generated from PID through a low pass filter.
        #       An example of a butterworth low pass filter is shown here, you can implement any filter you like. Before implementing the filter, look for the noise yourself and compare the output of unfiltered data and filtered data
        #       Filter adds delay to the signal, so there is a tradeoff between the noise rejection and lag. More lag is not good for controller as it will react little later.
        #       Alternatively, you can apply filter on the source of noisy data i.e. WhyCon position feedback instead of applying filter to the output of PID
        #       The filter implemented here is not the best filter, tune this filter that has the best noise rejection and less delay.

        # BUTTERWORTH FILTER
        span = 15
        for index, val in enumerate([roll, pitch, throttle]):
            PID_OUTPUT_VALUES[index].append(val)
            if len(PID_OUTPUT_VALUES[index]) == span:
                PID_OUTPUT_VALUES[index].pop(0)
            if len(PID_OUTPUT_VALUES[index]) != span-1:
                return
            order = 3
            fs = 60           # Sampling frequency (camera FPS)
            fc = 5            # Low pass cutoff frequency
            nyq = 0.5 * fs    # Nyquist frequency
            wc = fc / nyq
            b, a = scipy.signal.butter(
                N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
            filtered_signal = scipy.signal.lfilter(
                b, a, PID_OUTPUT_VALUES[index])
            if index == 0:
                self.rc_message.rc_roll = np.uint16(filtered_signal[-1])
            elif index == 1:
                self.rc_message.rc_pitch = np.uint16(filtered_signal[-1])
            elif index == 2:
                self.rc_message.rc_throttle = np.uint16(filtered_signal[-1])

        # Check the bounds of self.rc_message.rc_throttle, self.rc_message.rc_roll and self.rc_message.rc_pitch aftre rfiltering

        if self.rc_message.rc_roll > MAX_ROLL:
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_roll = MIN_ROLL

        # Similarly added bounds for pitch yaw and throttle
        if self.rc_message.rc_pitch > MAX_PITCH:
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_pitch = MIN_PITCH

        if self.rc_message.rc_throttle > MAX_THROTTLE:
            self.rc_message.rc_throttle = MAX_THROTTLE
        elif self.rc_message.rc_throttle < MIN_THROTTLE:
            self.rc_message.rc_throttle = MIN_THROTTLE

        # rospy.loginfo(self.rc_message)
        self.rc_pub.publish(self.rc_message)
        self.sum_error_publisherx.publish(self.sum_error[0])
        self.sum_error_publishery.publish(self.sum_error[1])
        self.sum_error_publisherz.publish(self.sum_error[2])
        self.errll.publish(-0.6)
        self.errul.publish(0.6)


    ##############################################################

    #Function Name:shutdown_hook
    #Input: none
    #Output: none
    #Logic: disarms the drone once rosnode will be terminated
    #Example call:shutdown_hook()


    def shutdown_hook(self):
        rospy.loginfo("Calling shutdown hook")
        rospy.loginfo("self.Kp  = "+ str(self.Kp))
        rospy.loginfo("self.Kd  = "+ str(self.Kd))
        rospy.loginfo("self.Ki  = "+ str(self.Ki))

        self.disarm()

    ##############################################################

    #Function Name:arm
    #Input: none
    #Output: none
    #Logic: arms the drone 
    #Example call:arm()

    def arm(self):
        rospy.loginfo("Calling arm service")
        rospy.sleep(3.0)
        service_endpoint = "/sentinel_drone/cmd/arming"
        rospy.wait_for_service(service_endpoint, 2.0)
        try:
            arming_service = rospy.ServiceProxy(service_endpoint, CommandBool)
            resp = arming_service(True)
            return resp.success, resp.result
        except rospy.ServiceException as err:
            rospy.logerr(err)

    ##############################################################

    #Function Name:disarm
    #Input: none
    #Output: none
    #Logic: disarms the drone
    #Example call:disarm()

    def disarm(self):
        rospy.loginfo("Calling disarm service")
        service_endpoint = "/sentinel_drone/cmd/arming"
        rospy.wait_for_service(service_endpoint, 10.0)
        try:
            arming_service = rospy.ServiceProxy(service_endpoint, CommandBool)
            resp = arming_service(False)
            return resp.success, resp.result
        except rospy.ServiceException as err:
            rospy.logerr(err)
        self.is_flying = False

    ##############################################################

    #Function Name:main
    #Input: none
    #Output: publishes the lat long values on the rostopic /geolocation by calling the g_ref_info function for georeferencing the images.
    #Logic: publishes the lat long values on the rostopic /geolocation by calling the g_ref_info function for georeferencing the images sequentially and saves the values one by one in each row in the data.csv file
    #Example call:main()

    def main(self):
        file_csv = open(self.csv_loc , "w") #reading csv
        file_csv.truncate()
        writer = csv.writer(file_csv)
        for i in range(1,num):
            self.g_ref_info(i) #calling the g_ref_info function for each image 
            ds = gdal.Open('/home/aniket/images/fupdated_task2d'+str(i)+'.tif')
            xoff, a, b, yoff, d, e = ds.GetGeoTransform()
            (x,y) = self.pixel2coord( 332,257 , a , b, xoff, d , e, yoff)
            self.mssssg.objectid = str(i) 
            self.mssssg.lat = np.float32(x)
            self.mssssg.long = np.float32(y)
            self.latlong.publish(self.mssssg) #publishing the message on rostopic /geolocation of the object_id and the lattitude and longitude detected
            writer.writerow([str(i), np.float32(x), np.float32(y)])
            rospy.loginfo("FINAL_OUTPUT  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
            print("FINAL_OUTPUT  "+str(i) +" "+ str(x) +" "+ str(y) )
        file_csv.close() #closing csv file
        exit() #terminating the entire program here


if __name__ == "__main__":

    controller = DroneController() #defining the object of the class DroneController
    controller.arm() #arming the drone 
    rospy.sleep(1)

    rate = rospy.Rate(55)
    rospy.loginfo("Entering PID controller loop")

    while not rospy.is_shutdown():

        controller.pid()

        if rospy.get_rostime().secs - controller.last_whycon_pose_received_at > 1: #if the whycon marker is not detected for more than 1 second, then logs the error in the terminal
            rospy.logerr("Unable to detect WHYCON poses")
       
        rate.sleep()
