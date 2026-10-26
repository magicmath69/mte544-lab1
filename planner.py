import numpy as np

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

# Type of trajectory
PARABOLA=0; SIGMOID=1


class planner:
    def __init__(self, type_, trajectory=PARABOLA):

        self.type=type_
        self.trajectory=trajectory


    def plan(self, goalPoint=[-1.0, -1.0]):

        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)

        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, num_points=30):
        # The return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # Both curves start at (0, 0), so reset the odometry before running
        # Scale x down here if the curve does not fit in the classroom space
        if self.trajectory==PARABOLA:
            # y = x^2 for x in [0.0, 1.5]
            x=np.linspace(0.0, 1.5, num_points)
            y=x**2

        elif self.trajectory==SIGMOID:
            # y = 2/(1 + e^(-2x)) - 1 for x in [0.0, 2.5]
            x=np.linspace(0.0, 2.5, num_points)
            y=2/(1 + np.exp(-2*x)) - 1

        return [[float(xi), float(yi)] for xi, yi in zip(x, y)]
