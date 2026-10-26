import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, PARABOLA, SIGMOID, planner
from controller import controller, trajectoryController

import os
import json
from datetime import datetime
from math import hypot

from pid import P, PD, PI, PID

# controller types accepted by --controller
CONTROLLER_TYPES={"P": P, "PD": PD, "PI": PI, "PID": PID}

# distance (m) from the goal at which the robot counts as arrived
GOAL_TOLERANCE=0.05

# Part 5: controller gains. p = proportional, v = derivative, i = integral;
# l = linear loop, a = angular loop. The controller type (--controller) picks which terms are used.
# retune on the real robot and write the final values here. Any gain can be overridden during the run (e.g. --kap 1.5).
POINT_GAINS={"klp": 0.8, "klv": 0.05, "kli": 0.1, "kap": 2.5, "kav": 0.25, "kai": 0.03}
TRAJECTORY_GAINS={"klp": 0.8, "klv": 0.05, "kli": 0.1, "kap": 3.0, "kav": 0.1, "kai": 0.02}

# goals are in the odometry frame, so a run should start at the odom origin.
START_POSE_TOLERANCE=0.05   # m
START_HEADING_TOLERANCE=0.1 # rad


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER, trajectory=PARABOLA,
                 controller_type=P, gains=None, log_dir="."):

        super().__init__("decision_maker")

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
        # gains come from POINT_GAINS / TRAJECTORY_GAINS above (plus any command-line overrides).
    
        if motion_type == POINT_PLANNER:
            gains=dict(POINT_GAINS, **(gains or {}))
            self.controller=controller(**gains, ctrl_type=controller_type, log_dir=log_dir)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            gains=dict(TRAJECTORY_GAINS, **(gains or {}))
            self.controller=trajectoryController(**gains, ctrl_type=controller_type, log_dir=log_dir)
            self.planner=planner(TRAJECTORY_PLANNER, trajectory)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # instantiate the localization, typ rawSensor (Odometry)
        self.localizer=localization(rawSensor, log_filename=os.path.join(log_dir, "robot_pose.csv"))

        # instantiate the planner
        # note: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.start_checked=False

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # TODO Part 3: run the localization node
        spin_once(self.localizer)    # note: this file is already running the decision_maker node.

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        if not self.start_checked:
            self.start_checked=True
            self.check_start_pose(self.localizer.getPose())

        vel_msg=Twist()
        
        # TODO Part 3: Check if you reached the goal
        if type(self.goal) == list:
            reached_goal=calculate_linear_error(self.localizer.getPose(), self.goal[-1]) < GOAL_TOLERANCE
        else: 
            reached_goal=calculate_linear_error(self.localizer.getPose(), self.goal) < GOAL_TOLERANCE
        

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #TODO Part 3: exit the spin
            raise SystemExit
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        #TODO Part 4: Publish the velocity to move the robot
        vel_msg.linear.x=velocity
        vel_msg.angular.z=yaw_rate
        self.publisher.publish(vel_msg)


    def check_start_pose(self, pose):
        # warn when the run does not start at the odometry origin (breaks planner's goals assumption).
        x, y, th=pose[0], pose[1], pose[2]
        print(f"start pose: x={x:.3f} m, y={y:.3f} m, theta={th:.3f} rad")
        if hypot(x, y) > START_POSE_TOLERANCE or abs(th) > START_HEADING_TOLERANCE:
            print("WARNING: the robot is not at the odometry origin, so the goals will be off. "
                  "Real robot: reset the odometry (ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose). "
                  "Simulation: use empty_world.launch.py (the house world spawns at (-2.0, -0.5)) or restart it.",
                  file=sys.stderr)


def make_run_dir(args, runs_dir):
    # one folder per run, so runs never overwrite each other.
    parts=[args.motion]
    if args.motion == "trajectory":
        parts.append(args.trajectory)
    parts.append(args.controller)
    parts.append(datetime.now().strftime("%Y%m%d-%H%M%S"))
    if args.name:
        parts.append(args.name)
    base=os.path.join(runs_dir, "_".join(parts))
    os.makedirs(runs_dir, exist_ok=True)
    # two runs started in the same second get _2, _3, ... instead of sharing a folder
    log_dir=base
    suffix=2
    while True:
        try:
            os.mkdir(log_dir)
            return log_dir
        except FileExistsError:
            log_dir=f"{base}_{suffix}"
            suffix+=1


import argparse


def build_arg_parser():

    argParser=argparse.ArgumentParser(description="point or trajectory")
    argParser.add_argument("--motion", type=str.lower, default="point", choices=["point", "trajectory"])
    argParser.add_argument("--trajectory", type=str.lower, default="parabola", choices=["parabola", "sigmoid"], help="parabola or sigmoid")
    argParser.add_argument("--controller", type=str.upper, default="P", choices=list(CONTROLLER_TYPES), help="P, PD, PI or PID")
    argParser.add_argument("--goal", type=float, nargs=2, default=[-1.0, -1.0], metavar=("X", "Y"), help="goal for --motion point (m)")
    for gain in POINT_GAINS:
        argParser.add_argument(f"--{gain}", type=float, default=None, help=f"override the {gain} gain for this run")
    argParser.add_argument("--name", type=str, default="", help="optional label added to the run folder name")
    argParser.add_argument("--runs-dir", type=str, default="runs", help="folder the run folders are created in")
    return argParser


def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    

    # Part 5: gains for this run = the defaults above, with any command-line overrides
    default_gains=POINT_GAINS if args.motion == "point" else TRAJECTORY_GAINS
    gains={name: (getattr(args, name) if getattr(args, name) is not None else value) for name, value in default_gains.items()}
    controller_type=CONTROLLER_TYPES[args.controller]

    # saves run settings
    log_dir=make_run_dir(args, args.runs_dir)
    with open(os.path.join(log_dir, "params.json"), "w") as file:
        json.dump({"motion": args.motion,
                   "trajectory": args.trajectory if args.motion == "trajectory" else None,
                   "goal": args.goal if args.motion == "point" else None,
                   "controller": args.controller,
                   "gains": gains,
                   "goal_tolerance": GOAL_TOLERANCE,
                   "name": args.name,
                   "started": datetime.now().isoformat(timespec="seconds")}, file, indent=2)
    print(f"{args.motion} run, {args.controller} controller, gains {gains}\nlogging to {log_dir}")

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion == "point":
        DM=decision_maker(Twist, '/cmd_vel', 10, goalPoint=args.goal, motion_type=POINT_PLANNER,
                          controller_type=controller_type, gains=gains, log_dir=log_dir)
    elif args.motion == "trajectory":
        DM=decision_maker(Twist, '/cmd_vel', 10, goalPoint=None, motion_type=TRAJECTORY_PLANNER,
                          trajectory=SIGMOID if args.trajectory == "sigmoid" else PARABOLA,
                          controller_type=controller_type, gains=gains, log_dir=log_dir)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    args = build_arg_parser().parse_args()

    main(args)
