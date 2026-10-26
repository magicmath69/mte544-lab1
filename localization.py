import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor, log_filename="robot_pose.csv"):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        # Best effort matches the /odom QoS on both the TurtleBot3 sim and the TurtleBot4.
        # depth=1 keeps only the newest message: decisions.py reads one message per 0.1 s
        # control step while odom arrives faster, so a deeper queue would hand the
        # controller a pose that is several messages (~0.5 s) old.
        odom_qos=QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        )
        
        self.loc_logger=Logger(log_filename, ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.odom_sub=self.create_subscription(odom, '/odom', self.odom_callback, odom_qos)

        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg):
        
        # TODO Part 3: Read x,y, theta, and record the stamp
        position=pose_msg.pose.pose.position
        q=pose_msg.pose.pose.orientation
        th=euler_from_quaternion([q.x, q.y, q.z, q.w])   # quaternion -> yaw (rad)
        self.pose=[position.x, position.y, th, pose_msg.header.stamp]

        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
if __name__=="__main__":
    init()
    LOC=localization(rawSensor)
    spin(LOC)
    
