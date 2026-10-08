# Lab 1 Commands

`X` = robot number.

## Every new terminal

```bash
source ~/robohub/turtlebot4/configs/.bashrc
export ROS_DOMAIN_ID=X
```

## Check topics

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic info /odom --verbose   # QoS
ros2 daemon stop                  # if topics are missing, then list again
```

## Undock / drive / dock

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # i , j l to move, k stop, z slower
ros2 action send_goal /dock irobot_create_msgs/action/Dock {}   # within 0.5 m of a dock
```

## Motions

Ctrl+C to stop.

```bash
python3 motions.py --motion circle   # 1 m wide, turns left
python3 motions.py --motion spiral   # grows to 1 m wide over 50 s
python3 motions.py --motion line     # 2 m at 15 s, 3 m at 20 s
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"   # if still moving after Ctrl+C
```

## Save data

Re-running a motion overwrites its CSVs, so move them first.

```bash
mkdir -p runs/circle_1 && mv *_content_circle.csv runs/circle_1/
wc -l runs/circle_1/*.csv
python3 filePlotter.py --files runs/circle_1/odom_content_circle.csv runs/circle_1/imu_content_circle.csv
```

## Map

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # terminal 1, drive slowly
ros2 launch turtlebot4_navigation slam.launch.py       # terminal 2
ros2 launch turtlebot4_viz view_robot.launch.py        # terminal 3
ros2 run nav2_map_server map_saver_cli -f map          # terminal 4, writes map.pgm + map.yaml
```

## Before leaving

```bash
ros2 action send_goal /dock irobot_create_msgs/action/Dock {}
zip -r lab1_data.zip runs map.pgm map.yaml
```

Copy the zip and a map screenshot off the PC, then delete everything.
