import sys
import time
from utilities import Logger

from rclpy.time import Time

from utilities import *
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters



rawSensors=0; kalmanFilter=1

# depth 1: only the newest message is kept, so the pose is not ~0.3-0.45 s old (have been doing this since lab 2)
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=1)


class localization(Node):

    # extra log columns after stamp: EKF heading and the odometry pose (EKF/IMU columns are nan with rawSensors) (as in previous lab)
    def __init__(self, type, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp",
                                                                        "kf_th", "odom_x", "odom_y", "odom_th"]):

        super().__init__("localizer")


        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None

        if type==rawSensors:
            self.initRawSensors();
        elif type==kalmanFilter:
            self.initKalmanfilter()
            self.kalmanInitialized = False
        else:
            print("We don't have this type for localization", sys.stderr)
            return

        self.timelast=time.time()

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initKalmanfilter(self):

        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)

        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)

        time_syncher.registerCallback(self.fusion_callback)


    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):

        if not self.kalmanInitialized:
            x=np.array([odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        euler_from_quaternion(odom_msg.pose.pose.orientation),
                        0,
                        0,
                        0])

            # TODO PART 5 Put the Q and R matrices
            # that you conclude from lab Three
            # Lab 3 values (per-sensor R used as it performed best), Q raised from Lab 3's 0.25*I to 0.375*I in Lab 4,
            # R taken from the Lab 1 sensor variances for [v (odom), w (odom), ax (IMU), ay (IMU)], P0 = Q
            Q=0.375*np.eye(6)
            R=np.diag([2e-5, 2e-4, 0.05, 0.06])
            P=Q.copy()

            self.kf=kalman_filter(P,Q,R, x)

            self.kalmanInitialized = True


        dt = time.time() - self.timelast

        self.timelast=time.time()


        z=np.array([odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.angular.z,
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y])

        self.kf.predict(dt)
        self.kf.update(z)

        xhat=self.kf.get_states()

        self.pose=np.array([xhat[0],
                            xhat[1],
                            normalize_angle(xhat[2]),
                            odom_msg.header.stamp])

        # x, y, th, w, v, vdot = xhat; predicted accelerations are vdot and v*w
        self.loc_logger.log_values([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                                    xhat[5], xhat[4]*xhat[3], xhat[4], xhat[3], xhat[0], xhat[1],
                                    Time.from_msg(odom_msg.header.stamp).nanoseconds,
                                    normalize_angle(xhat[2]),
                                    odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
                                    euler_from_quaternion(odom_msg.pose.pose.orientation)])

    def odom_callback(self, pose_msg):

        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

        nan=float("nan")
        self.loc_logger.log_values([nan]*8 + [Time.from_msg(self.pose[3]).nanoseconds, nan, self.pose[0], self.pose[1], self.pose[2]])


    def getPose(self):
        return self.pose


if __name__=="__main__":

    init()

    LOCALIZER=localization()


    spin(LOCALIZER)
