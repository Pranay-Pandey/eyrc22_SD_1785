#!/usr/bin/env python3

"""
Controller for the drone
"""


# standard imports
import copy
import time

# third-party imports
import scipy.signal
import numpy as np
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
from pid_tune.msg import PidTune
from sentinel_drone_driver.msg import PIDError, RCMessage
from sentinel_drone_driver.srv import CommandBool, CommandBoolResponse


MIN_THROTTLE = 1250             # Minimum throttle value, so that the drone does not hit the ground 
BASE_THROTTLE = 1330               # Base value of throttle for hovering. NOTE: Unlike simulator, the drone does not hover at 1500 value (HINT: Hovering value in hardware will range somewhere between 1280 and 1350). Also, the hovering thrust changes with battery % . Hence this will varry with time and the correct value will have to be generated using integral term to remove steady state error 
MAX_THROTTLE = 1400             # Maximum throttle value, so that the drone does not accelerate in uncontrollable manner and is in control. NOTE: DO NOT change this value, if changing, be very careful operating the drone 
SUM_ERROR_THROTTLE_LIMIT = 40    # Integral anti windup sum value. Decide this value by calcuating carefully


# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch


# for roll
BASE_ROLL = 1500
# roll = 100
MIN_ROLL = 1400
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 40

# for pitch
BASE_PITCH = 1510
# pitch = 100
MIN_PITCH = 1400
MAX_PITCH = 1600
SUM_ERROR_PITCH_LIMIT = 40




PID_OUTPUT_VALUES = [[], [], []] # This will be used to store data for filtering purpose

class DroneController:
    def __init__(self):

        self.BASE_THROTTLE = BASE_THROTTLE
        # self.throttle = throttle

        self.BASE_ROLL = BASE_ROLL
        # self.roll = roll

        self.BASE_PITCH = BASE_PITCH
        # self.pitch = pitch
        
        self.rc_message = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = None
        self.is_flying = False

        # Setpoints for x, y, z respectively      

        self.set_points = np.array([0, 0, 20])

        self.error = np.array([0, 0, 0])         # Error for roll, pitch and throttle
        # Create variables for previous error and sum_error
        self.previous_error = np.array([0, 0, 0])
        self.derivative_error = np.array([0, 0, 0])
        self.sum_error = np.array([0, 0, 0])
        self.sum_error_limit = np.array([SUM_ERROR_ROLL_LIMIT, SUM_ERROR_PITCH_LIMIT, SUM_ERROR_THROTTLE_LIMIT])


        # PID gains for roll, pitch and throttle with multipliers, so that you can copy the 
        # numbers from pid tune GUI slider as it is. For eg, Kp for roll on GUI slider is 100,
        # but multiplied by 0.1 in callback, then you can copy the number 100 instead of 0

        self.Kp = [0.175,  0,  13.54]

        # Create variables for Kd and Ki similarly
        self.Kd = [50, 0, 254.4]
        self.Ki = [0.0022, 0, 0.0263]

        # Initialize rosnode

        node_name = "controller"
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown_hook)

        # Create subscriber for WhyCon 

        rospy.Subscriber("/whycon/poses", PoseArray, self.whycon_poses_callback)


        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required

        rospy.Subscriber(
            "/pid_tuning_altitude", PidTune, self.pid_tune_throttle_callback
        )
        rospy.Subscriber(
            "/pid_tuning_roll", PidTune, self.pid_tune_roll_callback
        )
        rospy.Subscriber(
            "/pid_tuning_pitch", PidTune, self.pid_tune_pitch_callback
        )

        # Create publisher for sending commands to drone 

        self.rc_pub = rospy.Publisher(
            "/sentinel_drone/rc_command", RCMessage, queue_size=1
        )

        # Create publisher for publishing errors for plotting in plotjuggler 

        self.pid_error_pub = rospy.Publisher(
            "/sentinel_drone/pid_error", PIDError, queue_size=1
        )

        # Create publisher for x,y,z errors
        self.sum_error_publisherx = rospy.Publisher(
            "/sentinel_drone/error_x", Float64, queue_size=1)
        self.sum_error_publishery = rospy.Publisher(
            "/sentinel_drone/error_y", Float64, queue_size=1)
        self.sum_error_publisherz = rospy.Publisher(
            "/sentinel_drone/error_z", Float64, queue_size=1)


    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = rospy.get_rostime().secs
        self.drone_whycon_pose_array = msg
        
    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.Kp * 0.01
        self.Ki[2] = msg.Ki * 0.01
        self.Kd[2] = msg.Kd * 0.1
        # Similarly add Kd and Ki for throttle 

    def pid_tune_roll_callback(self, msg):
        self.Kp[0] = msg.Kp * 0.01
        self.Ki[0] = msg.Kp * 0.0001
        self.Kd[0] = msg.Kd * 0.01

    def pid_tune_pitch_callback(self, msg):
        self.Kp[1] = msg.Kp * 0.01
        self.Ki[1] = msg.Kp * 0.0001
        self.Kd[1] = msg.Kd * 0.01



    def pid(self):          # PID algorithm

        # # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        # self.error[0] = self.drone_whycon_pose_array.poses[0].position.x - self.set_points[0]
        # self.error[1] = self.drone_whycon_pose_array.poses[0].position.y - self.set_points[1]
        # self.error[2] = self.drone_whycon_pose_array.poses[0].position.z - self.set_points[2]
        
        # # Similarlt calculate error for y and z axes 

        # # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)

        # self.derivative_error = self.error - self.previous_error

        # self.output = (self.Kp*self.error) + (self.Kd*self.derivative_error) + (self.Ki*self.sum_error)

        # self.previous_error = self.error

        # self.sum_error += self.error

        # self.sum_error = np.fmin(self.sum_error_limit, np.fmax(-self.sum_error_limit, self.sum_error))

        # self.output.astype("int")


        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        self.error[0] = self.drone_whycon_pose_array.poses[0].position.x - \
            self.set_points[0]
        roll_derivative_error = self.error[0]-self.previous_error[0]
        self.previous_error[0] = self.error[0]
        roll_sum_error = self.sum_error[0]
        self.sum_error[0] += self.error[0]

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis
        self.error[1] = -self.drone_whycon_pose_array.poses[0].position.y + \
            self.set_points[1]
        # self.error[1] *= -1
        pitch_derivative_error = -self.error[1]+self.previous_error[1]
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

        # if self.integral[0] > SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = SUM_ERROR_ROLL_LIMIT
        # if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = -SUM_ERROR_ROLL_LIMIT

        self.rc_message.rc_roll = BASE_ROLL + \
            self.error[0]*self.Kp[0]+roll_derivative_error * \
            self.Kd[0]+roll_sum_error*self.Ki[0]

        self.rc_message.rc_pitch = BASE_PITCH + \
            self.error[1]*self.Kp[1]+pitch_derivative_error * \
            self.Kd[1]+pitch_sum_error*self.Ki[1]

        self.rc_message.rc_throttle = BASE_THROTTLE + \
            self.error[2]*self.Kp[2]+throttle_derivative_error * \
            self.Kd[2]+throttle_sum_error*self.Ki[2]
        # self.rc_message.rc_roll = BASE_ROLL + self.output[0]
        # self.rc_message.rc_pitch = BASE_PITCH + self.output[1]
        # self.rc_message.rc_throttle = BASE_THROTTLE + self.output[2]

        # Write the PID equations and calculate the self.rc_message.rc_throttle, self.rc_message.rc_roll, self.rc_message.rc_pitch

        
        # Send constant 1500 to rc_message.rc_yaw

        self.rc_message.rc_yaw = np.uint16(1500)

    #------------------------------------------------------------------------------------------------------------------------

        #publishing alt error, roll error, pitch error, drone message

        # self.rc_pub.publish(self.rc_message)

        self.publish_data_to_rpi(self.rc_message.rc_roll, self.rc_message.rc_pitch, self.rc_message.rc_throttle)
        # self.publish_data_to_rpi(roll, pitch, throttle)

        # Publish error messages for plotjuggler debugging 

        self.pid_error_pub.publish(
            PIDError(
                roll_error=self.error[0],
                pitch_error=self.error[1],
                throttle_error=self.error[2],
                zero_error=0,
            )
        )




    def publish_data_to_rpi(self, roll, pitch, throttle):

        #self.rc_message.rc_throttle = np.uint16(throttle)

        self.rc_message.rc_throttle = np.uint16(throttle)
        self.rc_message.rc_pitch = np.uint16(pitch)
        self.rc_message.rc_roll = np.uint16(roll)
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
            b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
            filtered_signal = scipy.signal.lfilter(b, a, PID_OUTPUT_VALUES[index])
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

        if self.rc_message.rc_throttle > MAX_THROTTLE:    
            self.rc_message.rc_throttle = MAX_THROTTLE
        elif self.rc_message.rc_throttle < MIN_THROTTLE:
            self.rc_message.rc_throttle = MIN_THROTTLE

        if self.rc_message.rc_pitch > MAX_PITCH:    
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_pitch = MIN_PITCH        

        # Similarly add bounds for pitch yaw and throttle 
        rospy.loginfo("RC_Message = "+  str(self.rc_message))
        self.rc_pub.publish(self.rc_message)
        self.sum_error_publisherx.publish(self.error[0])
        self.sum_error_publishery.publish(self.error[1])
        self.sum_error_publisherz.publish(self.error[2])

    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop 

    def shutdown_hook(self):
        rospy.loginfo("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone 

    def arm(self):
        rospy.loginfo("Calling arm service")
        service_endpoint = "/sentinel_drone/cmd/arming"
        rospy.wait_for_service(service_endpoint, 2.0)
        try:
            arming_service = rospy.ServiceProxy(service_endpoint, CommandBool)
            resp = arming_service(True)
            return resp.success, resp.result
        except rospy.ServiceException as err:
            rospy.logerr(err)

    # Function to disarm the drone 
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


if __name__ == "__main__":

    controller = DroneController()
    controller.arm()
    rospy.sleep(1)

    rospy.loginfo("Entering PID controller loop")
    while not rospy.is_shutdown():

        controller.pid()

        if rospy.get_rostime().secs - controller.last_whycon_pose_received_at > 1:
            rospy.logerr("Unable to detect WHYCON poses")
        
        # Add the sleep time to run the controller loop at desired rate
        #rospy.sleep()

