
from pid import PID_ctrl

import numpy as np

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:



    # formatting: log_prefix goes in front of the error logs (e.g. runs/<run>_linear.csv)
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, log_prefix=""):

        self.PID_linear=PID_ctrl(PID, klp, klv, kli, filename_=log_prefix + "linear.csv")
        self.PID_angular=PID_ctrl(PID, kap, kav, kai, filename_=log_prefix + "angular.csv")


    def vel_request(self, pose, goal, status):

        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)

        linear_vel = 0.5 if linear_vel > 1.0 else linear_vel
        angular_vel= 0.5 if angular_vel > 1.0 else angular_vel


        return linear_vel, angular_vel


class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, lookAhead=1.0, targetVel=1.0, log_prefix=""):
        super().__init__(klp, klv, kli, kap, kav, kai, log_prefix)
        self.lookAhead=lookAhead
        self.targetVelocity=targetVel

    def vel_request(self, pose, listGoals, status):

        goal=self.lookFarFor(pose, listGoals)

        finalGoal=listGoals[-1]


        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)

        linear_vel = 0.1 if linear_vel > 0.1 else linear_vel
        # limit both turning directions (provided code only limits left turns???)
        angular_vel = max(-0.5, min(0.5, angular_vel))


        return linear_vel, angular_vel


    def lookFarFor(self, pose, listGoals):

        poseArray=np.array([pose[0], pose[1]])
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]