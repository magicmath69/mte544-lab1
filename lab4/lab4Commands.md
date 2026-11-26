# Lab 4 Commands

Run everything from the `lab4/` folder. Handout: [README.md](README.md), marks: [rubrics.md](rubrics.md).

## Get the code
```bash
git clone -b LabFour https://github.com/magicmath69/mte544-lab1.git mte544
cd mte544/lab4
python3 -c "import sklearn"          # the cost map needs scikit-learn; if this errors: pip3 install scikit-learn
```

## Map
The planner reads `<map>.yaml` + `<map>.pgm` from this folder (default `room`, change with `--map`).
The map frame is where the robot was when SLAM started, and `mapPublisher.py` sets map = odom.
So the odometry must be reset at **the same spot** as when the map was made, before every run.

### Simulation (VM), pre-lab map of the house
Terminal 1:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
Terminal 2 (SLAM; if cartographer isn't installed try `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True`):
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
Terminal 3, drive around until the map is complete:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Terminal 4, save it as `house.pgm` / `house.yaml`:
```bash
ros2 run nav2_map_server map_saver_cli -f house
```
Close SLAM, then restart Gazebo (or `ros2 service call /reset_world std_srvs/srv/Empty {}`) so the robot and odometry are back at the start.
Check that `ros2 topic echo /odom --once` shows a position near (0, 0); if not, restart Gazebo.

### Robot (lab), at our entrance exactly on the tape
```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
# put the robot on the tape, then:
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose {}        # terminal 1
ros2 launch turtlebot4_navigation slam.launch.py                         # terminal 2
ros2 launch turtlebot4_viz view_robot.launch.py                          # terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard                     # terminal 4
ros2 run nav2_map_server map_saver_cli -f room                           # terminal 5 (rerun if "Failed to spin map subscription")
```
Show the map to a TA (Part 3). Don't pick up the robot afterwards; if you do: dock, undock, put it on the tape, reset the pose.

## Check a plan without driving (optional)
```bash
python3 planner.py --map room --goal 2.0 1.5          # start defaults to 0 0; plots both heuristics on the cost map
python3 planner.py --map room --start 0 0 --goal 2.0 1.5
```

## Changing the wall clearance (cost-map width)
The cost map is a Gaussian around every occupied map cell with standard deviation `laser_sig` (m).
A* treats cells with cost > 0.8 as walls (everything within about 0.67 × `laser_sig` of an obstacle)
and adds the cost of each cell it steps on, so paths also keep away from walls.
To change it, edit the number in `initTrajectoryPlanner` in `planner.py`:
```python
self.m_utilites=mapManipulator(filename_=self.mapName, laser_sig=0.4)
```
- Smaller (e.g. 0.2): narrower blocked band, use it if A* says "no path found" in narrow corridors.
- Larger: more clearance from walls, but doorways/corridors may close.
- `decisions.py` and `planner.py` (offline check) both use this value. `mapPublisher.py` (RViz) keeps the course default 0.01, so RViz shows the bare walls, not the blocked band.

## Runs (same in sim and lab, add `--map house` in sim)
Terminal 1, map for RViz:
```bash
python3 mapPublisher.py --map room
```
Terminal 2:
```bash
rviz2 -d pathPlanner.rviz
```
Terminal 3, one run per heuristic; set the goals with **2D Goal Pose** in RViz (two goals far apart, one after the other):
```bash
python3 decisions.py --map room --heuristic euclidean
python3 decisions.py --map room --heuristic manhattan
```
Other option: `--localizer odom` (raw odometry instead of the EKF). If no path is found, see "Changing the wall clearance" above.
Stop with Ctrl+C, then:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```
Before the next run: robot back on the tape + `ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose {}` (sim: reset/restart Gazebo).

Logs are saved as `runs/<date>-<time>_<heuristic>_<localizer>_{pose,path,linear,angular}.csv`.

## Plot
```bash
python3 plot_run.py --map room                        # newest run
python3 plot_run.py runs/<run>_pose.csv --map room    # a specific run
```

## Before leaving the lab
Copy `runs/` and `room.pgm` + `room.yaml` off the PC (USB or git push), then delete your files.
