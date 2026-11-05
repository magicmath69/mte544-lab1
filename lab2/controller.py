import os

import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

# Burger maxima: 0.22 m/s, 2.84 rad/s; TurtleBot4: 0.31 m/s, 1.90 rad/s.
MAX_LINEAR_VELOCITY=0.22
MAX_ANGULAR_VELOCITY=1.90

# Part 5: anti-windup clamp for the linear loop (see pid.py):
# note: angular error is signed, so its integral unwinds on its own.
LINEAR_INTEGRAL_LIMIT= None #0.25 * MAX_LINEAR_VELOCITY

# Improved point controller (pointController below), tested in simulation only (sim_point.py --improved)
POINT_INTEGRAL_BAND=np.radians(45)                      # integrators run only while the goal is this close to the heading
POINT_LINEAR_INTEGRAL_LIMIT=0.25 * MAX_LINEAR_VELOCITY  # most speed the linear integral may add (m/s)
POINT_ANGULAR_INTEGRAL_LIMIT=0.3                        # most turn rate the angular integral may add (rad/s)

class controller:
    
    
    # default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, ctrl_type=P, log_dir="."):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        # ctrl_type (P, PD, PI or PID) is used for both loops; the logs go to log_dir (one folder per run).
        self.PID_linear=PID_ctrl(ctrl_type, klp, klv, kli, filename_=os.path.join(log_dir, "linear.csv"),
                                 integral_limit=LINEAR_INTEGRAL_LIMIT)
        self.PID_angular=PID_ctrl(ctrl_type, kap, kav, kai, filename_=os.path.join(log_dir, "angular.csv"))

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # Part 4: clamp both signs so turning right is limited like turning left.
        linear_vel=float(np.clip(linear_vel, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY))
        angular_vel=float(np.clip(angular_vel, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY))
        
        return linear_vel, angular_vel


class pointController(controller):

    # Improved PID for point goals. With the controller above, PID circles the goal in simulation: the distance error
    # is never negative, so its integral only grows and holds the robot at full speed. Three changes fix this:
    #   1. signed distance error d*cos(e_theta): negative when the goal is behind, so the robot backs up instead of
    #      driving away, and the integral unwinds once the robot passes the goal (linear.csv logs this signed error);
    #   2. integral separation: both integrals are reset while the goal is more than POINT_INTEGRAL_BAND off the
    #      heading, so a large turn does not wind up the heading integral;
    #   3. anti-windup clamps on both integrals.
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, ctrl_type=P, log_dir="."):

        super().__init__(klp, klv, kli, kap, kav, kai, ctrl_type, log_dir)
        self.PID_linear.integral_limit=POINT_LINEAR_INTEGRAL_LIMIT
        self.PID_angular.integral_limit=POINT_ANGULAR_INTEGRAL_LIMIT

    def vel_request(self, pose, goal, status):

        e_ang=calculate_angular_error(pose, goal)
        e_lin=calculate_linear_error(pose, goal) * np.cos(e_ang)

        if abs(e_ang) > POINT_INTEGRAL_BAND:
            self.PID_linear.reset_integral()
            self.PID_angular.reset_integral()

        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)

        linear_vel=float(np.clip(linear_vel, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY))
        angular_vel=float(np.clip(angular_vel, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY))

        return linear_vel, angular_vel


class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, ctrl_type=P, log_dir="."):
        
        super().__init__(klp, klv, kli, kap, kav, kai, ctrl_type, log_dir)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # applys robot velocity limits when following a trajectory as well as pure pursuit
        linear_vel=float(np.clip(linear_vel, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY))
        angular_vel=float(np.clip(angular_vel, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY))
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
