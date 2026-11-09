import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensors, kalmanFilter

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, SPIRAL_4TUNE, planner
from controller import controller, trajectoryController

import os
from datetime import datetime

# logging folder, files named <date>-<time>_q<q>_r<r>.csv
RUNS_DIR="runs"
# fill log row with NaNs when no controller is run (spiral)
NO_CONTROLLER=[float("nan")]*6


class decision_maker(Node):
    
    
    # Q = q*I and R = r*I for the EKF. log_file is this runs log
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER, q=0.5, r=0.5, log_file="robotPose.csv"):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)
        
        
        publishing_period=1/rate

        self.reachThreshold=0.01


        # TODO Part 3: use the Kalman Filter
        # EKF runs once per timer tick, so its period is 0.1.
        self.localizer=localization(type=kalmanFilter, dt=publishing_period, loggerName=log_file, q=q, r=r)
        
        if motion_type==POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
            self.planner=planner(POINT_PLANNER)

        
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
            self.planner=planner(TRAJECTORY_PLANNER)
        elif motion_type==SPIRAL_4TUNE:
            self.controller=None
            self.planner=None
            self.linearVelocity = 0
        
        else:
            print("Error! you don't have this type of planner", file=sys.stderr)
            return

        self.motion_type=motion_type
        self.goal=self.planner.plan() if self.motion_type != SPIRAL_4TUNE else None
        

        self.create_timer(publishing_period, self.timerCallback)



    def timerCallback(self):
        
        spin_once(self.localizer)

        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        
        vel_msg=Twist()

        if self.motion_type == SPIRAL_4TUNE:
            self.linearVelocity += 0.01 if self.linearVelocity < 1.0 else 0.0
            vel_msg.linear.x=self.linearVelocity
            vel_msg.angular.z=1.0
            self.publisher.publish(vel_msg)
            self.localizer.log_row(NO_CONTROLLER)    # no controller in the spiral
            return

        
        if type(self.goal) == list:
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal[-1]) <self.reachThreshold else False
        else: 
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal) <self.reachThreshold else False


        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            # log final pos, replace the PID loggers save_log() calls
            self.localizer.log_row(NO_CONTROLLER)
            
            raise SystemExit
        
        velocity, yaw_rate = self.controller.\
            vel_request(self.localizer.getPose(), self.goal, True)

        # log this step's EKF row with the errors the controller just computed from that pose
        self.localizer.log_row(self.controller.PID_linear.last_log + self.controller.PID_angular.last_log)
        
        vel_msg.linear.x=velocity
        vel_msg.angular.z=yaw_rate
        
        self.publisher.publish(vel_msg)


import argparse
def main(args=None):
    
    
    init()
    
    # one log file per run
    os.makedirs(RUNS_DIR, exist_ok=True)
    log_file=os.path.join(RUNS_DIR, f"{datetime.now().strftime('%Y%m%d-%H%M%S')}_q{args.q:g}_r{args.r:g}.csv")
    print(f"{args.motion} run, Q = {args.q:g}*I, R = {args.r:g}*I, logging to {log_file}")
    run_settings=dict(q=args.q, r=args.r, log_file=log_file)
    
    if args.motion == "point":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=POINT_PLANNER, **run_settings)
    elif args.motion == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=TRAJECTORY_PLANNER, **run_settings)
    elif args.motion == "spiral":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=SPIRAL_4TUNE, **run_settings)
    else:
        print("invalid motion type", file=sys.stderr)
        return



    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")




if __name__=="__main__":
    
    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="spiral")
    # EKF covariances for this run: Q = q*I, R = r*I
    argParser.add_argument("--q", type=float, default=0.5, help="process noise, Q = q*I (default 0.5)")
    argParser.add_argument("--r", type=float, default=0.5, help="measurement noise, R = r*I (default 0.5)")
    args = argParser.parse_args()

    main(args)
