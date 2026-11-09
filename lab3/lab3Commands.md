# Lab 3 Commands

Run everything from the `lab3/` folder.

## Get the code
```bash
git clone -b LabThree https://github.com/magicmath69/mte544-lab1.git mte544
cd mte544/lab3
```

## Simulation (VM)
Terminal 1:
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
Terminal 2, before every run:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 service call /reset_world std_srvs/srv/Empty {}
```

## Robot (lab)
Before every run:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose
```

## Runs (same in both)
```bash
python3 decisions.py --motion spiral                 # Q = R = 0.5
python3 decisions.py --motion spiral --q 0.1
python3 decisions.py --motion spiral --q 5
python3 decisions.py --motion spiral --r 0.1
python3 decisions.py --motion spiral --r 5
python3 decisions.py --motion point                  # stops by itself at (-1, -1)
```
Spiral: stop with Ctrl+C after ~20 s, then send the stop command.

## Plot
```bash
python3 plot_run.py                                  # newest run
python3 plot_run.py runs/<file>.csv                  # a specific run
```
Logs are saved as `runs/<date>-<time>_q<q>_r<r>.csv`.

## Before leaving the lab
Copy `runs/` off the PC (USB or git push), then delete your files.
