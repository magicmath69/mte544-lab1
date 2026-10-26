# Lab 2 Commands

## Quick reference

| Goal | Command |
|---|---|
| Reset the position (robot) | `ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose` |
| Reset the position (Gazebo) | `ros2 service call /reset_world std_srvs/srv/Empty {}` |
| Stop the robot | Ctrl+C, then `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"` |
| Plot a run | `python3 plot_errors.py --files runs/<run>/robot_pose.csv` |

---

## Drive to a point (p controller)

1. **Reset the position** before every run. The goals are measured from where the robot was when the position was reset.
   ```bash
   ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose
   ```
2. **Run:**
   ```bash
   python3 decisions.py --motion point --controller P
   ```
3. **You should see:**
   - `point run, P controller, gains {...}` and `logging to runs/point_P_<date-time>`
   - `start pose: x=0.000 m, y=0.000 m, theta=0.000 rad`. A **WARNING** instead means the position wasn't reset: Ctrl+C, reset, run again.
   - `reached goal` once the robot is within 5 cm of the goal.
4. **Choose a different goal** (in metres, x forward, y left). The default is (−1, −1), 1 m behind and 1 m to the right:
   ```bash
   python3 decisions.py --motion point --controller P --goal 1.0 0.5
   ```

## Other controllers

Set `--controller` to `P`, `PD`, `PI` or `PID`. The same type is used for both driving and turning.
```bash
python3 decisions.py --motion point --controller PD
python3 decisions.py --motion point --controller PI
python3 decisions.py --motion point --controller PID
```

## Drive in NOT a straight line

1. **Reset the position,** then run:
   ```bash
   python3 decisions.py --motion trajectory --trajectory parabola --controller PD
   python3 decisions.py --motion trajectory --trajectory sigmoid --controller PD
   ```
2. Defaults:
   - Parabola: 1.5 m forward, 2.25 m to the left.
   - Sigmoid: 2.5 m forward, 1 m to the left.
3. `reached goal` prints when the robot is within 5 cm of the end of the curve.

## Tune gains

1. **Override gains for one run,** and label it with `--name`:
   ```bash
   python3 decisions.py --motion point --controller P --kap 1.5 --name kap1.5
   python3 decisions.py --motion point --controller PD --kap 2.0 --kav 0.3 --name kap2_kav0.3
   ```
   Gain names are `k` + `l` (linear/driving) or `a` (angular/turning) + `p` (P), `v` (D) or `i` (I). For example, `--kav` is the angular D gain.
2. **Suggested order:**
   1. P: raise `kap` until the heading just overshoots, then raise `klp`.
   2. PD: add `kav` to remove the overshoot.
   3. PI / PID: add a small `kli` and `kai`. Keep `kai` small: it builds up during the first big turn and leaves a small heading offset.

Overrides only apply to that run, and the code is not changed. See "save the final gains" to make them permanent.

## Save the final gains in the code

1. Copy the best run's gains (from its `params.json`) into `POINT_GAINS` or `TRAJECTORY_GAINS` near the top of `decisions.py`.
2. Run once without any `--k...` options to check the new defaults are used.

## Check a run's results

1. **Each run saves a folder** `runs/<motion>_<controller>_<date-time>[_name]/` with:
   - `linear.csv`, `angular.csv`: error, error rate, error integral and timestamp for the driving and turning controllers.
   - `robot_pose.csv`: x, y, theta and timestamp from odometry.
   - `params.json`: the controller, gains, goal and goal tolerance used.
2. **List the runs and see a run's settings:**
   ```bash
   ls runs
   cat runs/<run>/params.json
   ```
3. **Plot the run.** One window opens per file; close it to see the next. `robot_pose.csv` shows the path.
   ```bash
   python3 plot_errors.py --files runs/<run>/linear.csv runs/<run>/angular.csv runs/<run>/robot_pose.csv
   ```

## Stop

1. Ctrl+C in the terminal running `decisions.py`.
2. If it keeps moving:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
   ```

---

## For testing in VM

1. **Use the empty world.** The TurtleBot3 simulator's position is measured from the world origin, and the house world starts the robot away from it.
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```
2. **Run `decisions.py` exactly as on the robot,** but reset the position with:
   ```bash
   ros2 service call /reset_world std_srvs/srv/Empty {}
   ```
   
## If something goes wrong

| What you see | What to do |
|---|---|
| `WARNING: the robot is not at the odometry origin` | Ctrl+C, reset the position, run again |
| Nothing happens after `logging to ...` | No odometry is arriving. Check with `python3 localization.py`: it should keep running and fill `robot_pose.csv`. Stop it with Ctrl+C |
| The robot circles the goal and never finishes | Stop it; raise `kap` or lower `kli` |
| The robot wobbles left and right | Lower `kap` or raise `kav` |
| `No such file or directory` when plotting | Check the folder name with `ls runs`; the file is `robot_pose.csv` |

## Reference: `decisions.py` options

| Option | Values | Default |
|---|---|---|
| `--motion` | `point`, `trajectory` | `point` |
| `--trajectory` | `parabola`, `sigmoid` | `parabola` |
| `--controller` | `P`, `PD`, `PI`, `PID` | `P` |
| `--goal X Y` | goal for `point`, in metres | `-1 -1` |
| `--klp --klv --kli --kap --kav --kai` | override one gain for this run | values in `decisions.py` |
| `--name` | label added to the run folder | none |
| `--runs-dir` | folder to save runs in | `runs` |
