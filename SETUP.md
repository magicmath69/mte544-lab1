# MTE544 Lab 1 workspace

Starter materials for sensor data processing with a simulated TurtleBot3 Burger.
The Python files contain unfinished TODOs; complete them before running the
motion experiments.

## References

- [Lab manual](README.md)
- [Grading rubric](rubrics.md)
- [Simulation launch guide](tbt3Simulation.md)
- [Physical TurtleBot4 connection guide](connectToUWtb4s.md)

The starter materials originate from
[UW-MTE544/MTE544_student](https://github.com/UW-MTE544/MTE544_student),
LabOne branch. The original MIT license and attribution are retained.

## Environment

Use the course Ubuntu environment with its matching ROS 2 and Gazebo versions.
The simulation guide assumes the course dependencies are already installed.
Required components include Python 3, rclpy, the ROS message packages,
TurtleBot3 simulation and teleoperation packages, Matplotlib, SLAM Toolbox,
RViz2, and the Nav2 map server. Source ROS and the course workspace in each
terminal.

## Files to complete

1. `utilities.py`: CSV logging and quaternion-to-yaw conversion. Check that
   its file reader preserves every data row and supports the laser CSV format.
2. `motions.py`: imports, publisher, compatible QoS, three subscriptions,
   sensor callbacks and readiness flags, and the three motion functions.
3. `filePlotter.py`: required odometry, IMU, and Cartesian laser plots with
   titles, axis labels and units, legends, and grids. Convert timestamp
   differences from nanoseconds to seconds.

`image_viz.py` is a TurtleBot4 camera viewer and is not required by the listed
Lab 1 plot requirements.

## Simulation workflow

Launch Gazebo:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

In another sourced terminal, inspect the topics and their QoS:

```bash
ros2 topic list -t
ros2 topic info /odom --verbose
ros2 topic info /imu --verbose
ros2 topic info /scan --verbose
```

After completing the code, run each motion separately from this directory:

```bash
python3 motions.py --motion circle
python3 motions.py --motion spiral
python3 motions.py --motion line
```

The loggers create three CSV files per motion in the current working directory.
Preserve each run before repeating it: existing files for that motion are
overwritten. Stop keyboard teleoperation while the motion script controls the
robot. Implement a zero-velocity command when an experiment ends.

For mapping, follow Part 8 of the lab manual. Launch SLAM and RViz, drive slowly
with keyboard teleoperation, and save the map:

```bash
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true
ros2 launch turtlebot3_bringup rviz2.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
ros2 run nav2_map_server map_saver_cli -f map
```

Run these commands in separate sourced terminals as appropriate. Keep Gazebo,
SLAM, RViz, and teleoperation running during map acquisition. Save `map.pgm`,
`map.yaml`, and a screenshot of the acquired map.

## Deliverables

- A first version of completed code 24 hours before the lab section.
- One report PDF: cover page plus at most two content pages containing the
  required plots, map screenshot, explanations, and discussion.
- One ZIP of commented code and recorded CSVs; preserve the saved map files too.

Consult the manual and rubric for physical-robot checkoffs and group submission
requirements. Simulation is explicitly allowed for mapping and supports
pre-lab testing; the manual does not waive all physical demonstrations.
