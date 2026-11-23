
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    # mapName = map file without extension (room -> room.yaml + room.pgm)
    # heuristic: must be "manhattan" or "euclidean" for A*
    def __init__(self, type_, mapName="room", heuristic="euclidean"):

        self.type=type_
        self.mapName=mapName
        self.heuristic=heuristic

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid.
        # cells with cost > 0.8 are walls for A*. grid as 0.3 m blocks ~0.2 m, about the robot's radius
        self.m_utilites=mapManipulator(filename_=self.mapName, laser_sig=0.3)
            
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        # A* on the cost map, result is a list of cells from start to end
        cellPath = search(self.costMap, startPose, endPose, self.heuristic)

        if cellPath is None:
            return None

        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        
        Path = list(map(self.m_utilites.cell_2_position, cellPath))



        # TODO PART 5 return the path as list of [x,y]
        return [[x, y] for x, y in Path]




if __name__=="__main__":

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    import argparse, time
    import rclpy
    import matplotlib.pyplot as plt

    parser=argparse.ArgumentParser(description="plan offline with both heuristics and plot the paths")
    parser.add_argument("--map", type=str, default="room")
    parser.add_argument("--start", type=float, nargs=2, default=[0.0, 0.0], help="x y in m")
    parser.add_argument("--goal", type=float, nargs=2, required=True, help="x y in m")
    args=parser.parse_args()

    rclpy.init()   # mapManipulator is a ROS node

    # same cost map as the robot uses (laser_sig set in initTrajectoryPlanner)
    offline_planner=planner(TRAJECTORY_PLANNER, args.map)
    offline_planner.initTrajectoryPlanner()
    m_utilites=offline_planner.m_utilites

    map_likelihood=offline_planner.costMap

    res=m_utilites.getResolution(); ox, oy=m_utilites.getOrigin()
    extent=[ox, ox + m_utilites.width*res, oy, oy + m_utilites.height*res]
    plt.imshow(map_likelihood, cmap="gray_r", extent=extent, vmin=0, vmax=1)

    for heuristic, color in [("manhattan", "tab:blue"), ("euclidean", "tab:red")]:
        start_time=time.time()
        cellPath=search(map_likelihood, m_utilites.position_2_cell(args.start), m_utilites.position_2_cell(args.goal), heuristic)
        if cellPath is None:
            continue
        xy=np.array(list(map(m_utilites.cell_2_position, cellPath)))
        length=np.sum(np.hypot(*np.diff(xy, axis=0).T))
        print(f"{heuristic}: {len(cellPath)} cells, {length:.2f} m, {time.time() - start_time:.2f} s to plan")
        plt.plot(xy[:, 0], xy[:, 1], color=color, label=f"{heuristic} ({length:.2f} m)")

    plt.plot(*args.start, "go", label="start")
    plt.plot(*args.goal, "rx", markersize=10, label="goal")
    plt.title(f"A* on {args.map}, laser_sig {m_utilites.laser_sig} m")
    plt.xlabel("x (m)"); plt.ylabel("y (m)")
    plt.legend(); plt.grid(True)
    plt.show()
