

import sys
import os
from datetime import datetime


from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error, Logger
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensors, kalmanFilter

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController


from geometry_msgs.msg import PoseStamped


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

RUNS_DIR="runs"

class decision_maker(Node):

    # heuristic / mapName go to the A* planner, localizer_type is rawSensors or kalmanFilter,
    # run_name is the prefix of this run's logs for formatting: (e.g. runs/<date>-<time>_<heuristic>_<localizer>)
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER,
                 heuristic="euclidean", mapName="room", localizer_type=kalmanFilter, run_name="run"):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)


        self.create_subscription(PoseStamped, "/goal_pose", self.designPathFor, 10)

        self.pathPublisher = self.create_publisher(Path, '/designedPath', 10)

        publishing_period=1/rate

        # TODO PART 5 choose your threshold
        # 5 cm chosen, same as in lab 2. We also had errors sometimes with 1cm, and TA's have confirmed 5cm is reasonable for TB4.
        self.reachThreshold=0.05

        # TODO PART 5 your localization type
        # EKF with the Lab 3 Q and R (localization.py) by default; "--localizer odom" switches to rawSensors
        self.localizer=localization(localizer_type, loggerName=run_name + "_pose.csv")

        # logging of all planned paths. goal = how many goals were set before it in this run
        self.path_logger=Logger(run_name + "_path.csv", ["goal", "x", "y"])
        self.goal_count=0



        if motion_type==POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6, log_prefix=run_name + "_")
            self.planner=planner(POINT_PLANNER)


        elif motion_type==TRAJECTORY_PLANNER:
            # TODO PART 5 Put the gains that you concluded from lab 2
            # runs from which they were taken: 18:17:44 parabola and 18:27:45 sigmoid
            self.controller=trajectoryController(klp=0.2, klv=0.05, kli=0.025, kap=1.0, kav=0.2, kai=0.025,
                                                 log_prefix=run_name + "_")
            self.planner=planner(TRAJECTORY_PLANNER, mapName=mapName, heuristic=heuristic)

        else:
            print("Error! you don't have this type of planner", file=sys.stderr)


        self.goal = None

        self.create_timer(publishing_period, self.timerCallback)


        print("waiting for your input position, use 2D nav goal in rviz2")




    # This is for the rviz2 interface
    def designPathFor(self, msg: PoseStamped):

        spin_once(self.localizer)

        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return

        self.goal=self.planner.plan([self.localizer.getPose()[0], self.localizer.getPose()[1]],
                                     [msg.pose.position.x, msg.pose.position.y])

        # error handling: no path -> goal (or the robot) is inside a wall / the inflated area, or the goal is off the map
        if self.goal is None:
            print("no path to this goal, pick another one (or lower laser_sig in planner.py)")
            return

        # log the planned path (only the trajectory planner returns a list of points)
        if isinstance(self.goal, list) and isinstance(self.goal[0], list):
            self.goal_count+=1
            for x, y in self.goal:
                self.path_logger.log_values([self.goal_count, x, y])
            print(f"goal {self.goal_count}: path with {len(self.goal)} points, ends at ({self.goal[-1][0]:.2f}, {self.goal[-1][1]:.2f})")


    def timerCallback(self):

        spin_once(self.localizer)

        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return


        vel_msg=Twist()

        if self.goal is None:
            return

        if type(self.goal) == list:
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal[-1]) <self.reachThreshold else False
        else:
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal) <self.reachThreshold else False




        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)

            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()



            self.goal = None
            print("waiting for the new position input, use 2D nav goal on map")

            return

        velocity, yaw_rate = self.controller.\
            vel_request(self.localizer.getPose(), self.goal, True)


        vel_msg.linear.x=velocity
        vel_msg.angular.z=yaw_rate

        self.publisher.publish(vel_msg)
        self.publishPathOnRviz2(self.goal)



    def publishPathOnRviz2(self, path):

        Path_ =  Path()

        Path_.header.frame_id ="map"
        Path_.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            # Set the orientation of the pose. Here, it's set to a default orientation.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            Path_.poses.append(pose)

        self.pathPublisher.publish(Path_)

import argparse
def main(args=None):


    init()

    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

    # only one set of logs per run cuz more is annoying to look at data of in lab
    os.makedirs(RUNS_DIR, exist_ok=True)
    run_name=os.path.join(RUNS_DIR, f"{datetime.now().strftime('%Y%m%d-%H%M%S')}_{args.heuristic}_{args.localizer}")
    localizer_type=kalmanFilter if args.localizer == "ekf" else rawSensors
    settings=dict(heuristic=args.heuristic, mapName=args.map, localizer_type=localizer_type, run_name=run_name)

    if args.motion == "point":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=POINT_PLANNER, **settings)
    elif args.motion == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=TRAJECTORY_PLANNER, **settings)
    else:
        print("invalid motion type", file=sys.stderr)
        return

    print(f"map {args.map}, {args.heuristic} heuristic, {args.localizer} localizer, logging to {run_name}_*.csv")



    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")




if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point or trajectory")
    argParser.add_argument("--motion", type=str, default="trajectory")
    argParser.add_argument("--heuristic", type=str, default="euclidean", choices=["euclidean", "manhattan"], help="A* heuristic")
    argParser.add_argument("--map", type=str, default="room", help="map name without extension (room -> room.yaml, room.pgm)")
    argParser.add_argument("--localizer", type=str, default="ekf", choices=["ekf", "odom"], help="EKF (Lab 3 Q, R) or raw odometry")
    args = argParser.parse_args()

    main(args)
