# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Done
# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

# Motion parameters, tune these so the robot fits in the available space.
# MAX_LIN_VEL is kept under the TurtleBot3 Burger limit (0.22 m/s) so the same values work in sim and on the TurtleBot4
TIMER_PERIOD=0.1            # [s] period of timer_callback, i.e. time between two velocity commands
MAX_LIN_VEL=0.2             # [m/s] linear velocity of the circle, and the cap for the spiral and the line
CIRCLE_ANG_VEL=0.4          # [rad/s] circle radius is MAX_LIN_VEL/CIRCLE_ANG_VEL = 0.5 m
SPIRAL_ANG_VEL=0.4          # [rad/s] spiral stops growing at radius MAX_LIN_VEL/SPIRAL_ANG_VEL = 0.5 m
SPIRAL_RADIUS_RATE=0.01     # [m/s] how fast the spiral radius grows
LINE_ACC=0.02               # [m/s^2] acceleration along the line

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        self.lin_vel_=0.0

        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False

        # done
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        # keeps the last 10 messages
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', 10) 
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_min", "angle_increment", "stamp"])

        # done
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        # also last 10 recorded
        # best effort good for tb4, also works with tb3
        qos=QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        )

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.imu_sub=self.create_subscription(Imu, '/imu', self.imu_callback, qos)

        # ENCODER subscription
        self.odom_sub=self.create_subscription(Odometry, '/odom', self.odom_callback, qos)

        # LaserScan subscription
        self.laser_sub=self.create_subscription(LaserScan, '/scan', self.laser_callback, qos)
        
        self.create_timer(TIMER_PERIOD, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        """Log linear acceleration x/y, angular velocity z and the timestamp (ns)."""
        self.imu_initialized=True   # needed, or the motion timer never starts
        acc_x=imu_msg.linear_acceleration.x
        acc_y=imu_msg.linear_acceleration.y
        angular_z=imu_msg.angular_velocity.z
        stamp=Time.from_msg(imu_msg.header.stamp).nanoseconds
        self.imu_logger.log_values([acc_x, acc_y, angular_z, stamp])

    def odom_callback(self, odom_msg: Odometry):
        """Log x, y position, yaw (th) and the timestamp (ns)."""
        self.odom_initialized=True
        position=odom_msg.pose.pose.position
        q=odom_msg.pose.pose.orientation
        th=euler_from_quaternion([q.x, q.y, q.z, q.w])   # quaternion -> yaw (rad)
        stamp=Time.from_msg(odom_msg.header.stamp).nanoseconds
        self.odom_logger.log_values([position.x, position.y, th, stamp])

    def laser_callback(self, laser_msg: LaserScan):
        """Log one scan per row: all ranges, then angle_min, angle_increment, stamp (ns)."""
        self.laser_initialized=True
        stamp=Time.from_msg(laser_msg.header.stamp).nanoseconds
        row=list(laser_msg.ranges) + [laser_msg.angle_min, laser_msg.angle_increment, stamp]
        self.laser_logger.log_values(row)
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        
        msg=Twist()
        # Constant linear and angular velocity gives a circle of radius v/w,
        # positive angular.z turns the robot counter-clockwise
        msg.linear.x=MAX_LIN_VEL
        msg.angular.z=CIRCLE_ANG_VEL
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        # Constant angular velocity with a radius that grows on every timer tick: since v = r*w
        # the linear velocity ramps up and the circle opens into a spiral.
        # Once v reaches MAX_LIN_VEL the radius stops growing and the robot holds that circle
        self.radius_=min(self.radius_+SPIRAL_RADIUS_RATE*TIMER_PERIOD, MAX_LIN_VEL/SPIRAL_ANG_VEL)
        msg.linear.x=self.radius_*SPIRAL_ANG_VEL
        msg.angular.z=SPIRAL_ANG_VEL
        return msg

    def make_acc_line_twist(self):
        msg=Twist()
        # No angular velocity keeps the robot straight, the linear velocity is increased on every
        # timer tick to get a constant acceleration until it saturates at MAX_LIN_VEL
        self.lin_vel_=min(self.lin_vel_+LINE_ACC*TIMER_PERIOD, MAX_LIN_VEL)
        msg.linear.x=self.lin_vel_
        msg.angular.z=0.0
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
