import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):
    
    # loggerHeaders = the template's 9 columns, then the EKF heading, odometry (pose and twist) and the point controller's [e, e_dot, e_int] for its linear and angular loops (nan when unused).
    # q and r scale the identity matrices for Q and R (set with --q and --r in decisions.py).
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp",
                                                                            "kf_th", "odom_x", "odom_y", "odom_th", "odom_v", "odom_w",
                                                                            "lin_e", "lin_e_dot", "lin_e_int", "ang_e", "ang_e_dot", "ang_e_int"],
                 q=0.5, r=0.5):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)    # fixed typo in the template ("elf.loc_logger")
        self.pose=None
        self.row=None    # latest EKF row, to be written in by log_row()
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt, q, r)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt, q=0.5, r=0.5):
        
        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)
        
        # initial state [x, y, th, w, v, vdot] = 0
        x= np.zeros(6)
        
        # process noise covariance (6x6):
        Q= q*np.eye(6)

        # measurement noise covariance (4x4) for [v, w, ax, ay]
        R= r*np.eye(4)
        
        # initial covariance: start equally uncertain as one step of process noise; typ only affects the first ~2 s
        P= Q.copy() 
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # TODO Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # TODO Part 3: Use the EKF to perform state estimation
        # take measurement of linear velocity and angular velocity from odom msg, and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to get x,y and filter ax,ay
        # z = [v, w, ax, ay]. The odom twist is in the robot frame, so linear.x is the forward speed.
        z=np.array([odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.angular.z,
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y])
        
        # implement the two steps for estimation
        self.kf.predict()    # predict with motion model
        self.kf.update(z)    # correct with measurements
        
        xhat=self.kf.get_states()

        # update pose estimate to be returned by getPose
        # [x, y, th, stamp], same layout as odom_callback; pid.py reads the stamp (pose[3]) as a ROS time message.
        # dtype=object cuz the stamp is a message, not a number.
        self.pose=np.array([xhat[0], xhat[1], xhat[2], odom_msg.header.stamp], dtype=object)

        # TODO Part 4: log your data
        self.row=[imu_msg.linear_acceleration.x,
                  imu_msg.linear_acceleration.y,
                  xhat[5],
                  xhat[4]*xhat[3],
                  xhat[4],
                  xhat[3],
                  xhat[0],
                  xhat[1],
                  Time.from_msg(odom_msg.header.stamp).nanoseconds,
                  xhat[2],
                  odom_msg.pose.pose.position.x,
                  odom_msg.pose.pose.position.y,
                  euler_from_quaternion(odom_msg.pose.pose.orientation),
                  odom_msg.twist.twist.linear.x,
                  odom_msg.twist.twist.angular.z]

    # write the latest row
    def log_row(self, controller_values):
        if self.row is None:
            return
        self.loc_logger.log_values(self.row + list(controller_values))
        self.row=None
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
