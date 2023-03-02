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
from geometry_msgs.msg import PoseArray
from pid_tune.msg import PidTune
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from sentinel_drone_driver.msg import PIDError, RCMessage
from sentinel_drone_driver.srv import CommandBool, CommandBoolResponse


# Minimum throttle value, so that the drone does not hit the ground
MIN_THROTTLE = 1250
# Base value of throttle for hovering. NOTE: Unlike simulator, the drone does not hover at 1500 value (HINT: Hovering value in hardware will range somewhere between 1280 and 1350). Also, the hovering thrust changes with battery % . Hence this will varry with time and the correct value will have to be generated using integral term to remove steady state error
BASE_THROTTLE = 1300
MAX_THROTTLE = 1400          # Maximum throttle value, so that the drone does not accelerate in uncontrollable manner and is in control. NOTE: DO NOT change this value, if changing, be very careful operating the drone
# Integral anti windup sum value. Decide this value by calcuating carefully
SUM_ERROR_THROTTLE_LIMIT = 6000


# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch

# for roll
MIN_ROLL = 1400
BASE_ROLL = 1510
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 2000

# for pitch
MIN_PITCH = 1400
BASE_PITCH = 1510
MAX_PITCH = 1600
SUM_ERROR_PITCH_LIMIT = 2000


# This will be used to store data for filtering purpose
PID_OUTPUT_VALUES = [[], [], []]


class DroneController:
    def __init__(self):

        self.rc_message = RCMessage()
        self.rc_message_filtered = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = None
        self.is_flying = False
        self.lastime = time.time()
        self.last = 0
        self.land = [[0,0,20],[0,0,22],[0,0,24],[0,0,26],[0,0,27]]
        self.flag = False
        self.Landing = False
        self.bridge = CvBridge()


        self.lower = np.array([20, 100, 163])
        self.upper = np.array([39, 189, 232])

        # Setpoints for x, y, z respectively
        self.set_points = [0, 0, 20]

        self.error = [0, 0, 0]         # Error for roll, pitch and throttle

        # Create variables for previous error and sum_error
        self.previous_error = [0, 0, 0]
        self.sum_error = [0, 0, 0]

        # PID gains for roll, pitch and throttle with multipliers, so that you can copy the numbers from pid tune GUI slider as it is. For eg, Kp for roll on GUI slider is 100, but multiplied by 0.1 in callback, then you can copy the number 100 instead of 0
        self.Kp = [5.5,  -5.5,  13.54]

        # Create variables for Kd and Ki similarly
        self.Kd = [700, -700, 254.4]
        self.Ki = [0.0, -0.02, 0.0263]
        # Initialize rosnode

        node_name = "controller"
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown_hook)

        # Create subscriber for WhyCon

        rospy.Subscriber("/whycon/poses", PoseArray,
                         self.whycon_poses_callback)

        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required

        # rospy.Subscriber(
        #     "/pid_tuning_altitude", PidTune, self.pid_tune_throttle_callback
        # )
        rospy.Subscriber(
            "/pid_tuning_roll", PidTune, self.pid_tune_roll_callback
        )
        rospy.Subscriber(
            "/pid_tuning_pitch", PidTune, self.pid_tune_pitch_callback
        )

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
        # Similarly add Kd and Ki for throttle


    def Change_setpoint(self):

        self.now = time.time()
        if not(self.Landing):
            if self.now - self.lastime >= 5:
                self.set_points= [-5.5, 1, 20]
        else:
            self.set_points = self.land[0]
            if (self.last ==0):
                self.last = time.time()
            if self.now - self.last >= 1:
                if len(self.land) > 0:
                    self.land.pop(0)
                    self.last = 0
                else:
                    self.disarm()
                    exit(0)

    def Image_processing(self, img):

        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,self.lower,self.upper)

        cont, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if cont:
            cnt = max(cont, key=cv2.contourArea)
            if cv2.contourArea(cnt) >= 50:
                M = cv2.moments(cnt)
                cX, cY = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                print("detected block: ",cX,cY)

        



    def Image_Callback(self, msg):

        self.img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.Image_processing(self.img)
        cv2.imshow("img",self.img)
        cv2.waitKey(1)


    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = rospy.get_rostime().secs
        self.drone_whycon_pose_array = msg

    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.Kp * 0.01
        self.Ki[2] = msg.Kp * 0.0001
        self.Kd[2] = msg.Kd * 0.1

    def pid_tune_roll_callback(self, msg):
        self.Kp[0] = msg.Kp * 0.01
        self.Ki[0] = msg.Kp * 0.0001
        self.Kd[0] = msg.Kd * 0.01

    def pid_tune_pitch_callback(self, msg):
        self.Kp[1] = -msg.Kp * 0.01
        self.Ki[1] = -msg.Kp * 0.0001
        self.Kd[1] = -msg.Kd * 0.01

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

        self.Change_setpoint()

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
        
        

    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C.
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop

    def shutdown_hook(self):
        rospy.loginfo("Calling shutdown hook")
        rospy.loginfo("self.Kp  = "+ str(self.Kp))
        rospy.loginfo("self.Kd  = "+ str(self.Kd))
        rospy.loginfo("self.Ki  = "+ str(self.Ki))

        self.disarm()

    # Function to arm the drone

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

    rate = rospy.Rate(55)
    rospy.loginfo("Entering PID controller loop")
    count = 0
    flg = 0
    while not rospy.is_shutdown():

        controller.pid()

        if rospy.get_rostime().secs - controller.last_whycon_pose_received_at > 1:
            rospy.logerr("Unable to detect WHYCON poses")
        # Add the sleep time to run the c/ontroller loop at desired rate
        # if count>100 and count <200:
        #     controller.Kd[2]+=3
        # print(controller.Kd[2])
        # if flg==0:
        #     count+=1
        rate.sleep()
