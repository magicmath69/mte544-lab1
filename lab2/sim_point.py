"""Simulate a point run with our controller code (controller.py, pid.py) on a model of the TurtleBot 4.

The robot model is a unicycle whose speed and turn rate follow the commands after a dead time and a
first-order lag, reach only part of the commanded value, and never drop below a minimum speed. These
numbers were fitted to the logged pose of the P run 17:12:26 (runs_submit/point_P_20261028-171226_2).

The output goes to sim_runs/ in the same CSV format as a robot run, so lab2.ipynb and plot_errors.py
read it as is. It is simulated data: time stamps start at 0 and params.json says "source": "simulation".

    python3 sim_point.py --like runs_submit/point_P_20261028-171226_2 --controller PID

--gains-from takes the gains from another run instead, e.g. the PID gains of our last robot run:

    python3 sim_point.py --like runs_submit/point_P_20261028-170124_1 --controller PID \
        --gains-from runs_submit/trajectory_sigmoid_PID_20261028-182745 --name final_gains
"""
import argparse
import json
import os
import sys
import types
from datetime import datetime
from math import atan2, copysign, cos, sin

import pandas as pd

# pid.py only uses rclpy for Time.from_msg(stamp).nanoseconds; here the stamps are plain ints in ns.
class _Time:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    @staticmethod
    def from_msg(stamp):
        return _Time(stamp)

sys.modules["rclpy"] = types.ModuleType("rclpy")
sys.modules["rclpy.time"] = types.ModuleType("rclpy.time")
sys.modules["rclpy.time"].Time = _Time

from controller import controller, pointController
from pid import P, PD, PI, PID
from utilities import Logger, calculate_linear_error

CONTROLLER_TYPES = {"P": P, "PD": PD, "PI": PI, "PID": PID}
GAIN_NAMES = ["klp", "klv", "kli", "kap", "kav", "kai"]

# Robot model, fitted to the P run 17:12:26
DELAY = 0.25   # s, from a command to the robot starting to respond
TAU = 0.20     # s, first-order lag of the speed and the turn rate
V_RATIO = 0.9  # odometry speed / commanded speed
W_RATIO = 0.9  # odometry turn rate / commanded turn rate
V_MIN = 0.03   # m/s, the robot does not drive slower than this when asked to move

RATE = 10      # Hz, control loop (decisions.py)
DT = 0.001     # s, integration step


def simulate(gains, controller_type, goal, start, tolerance, max_time, log_dir, controller_class=controller):
    """Run the control loop of decisions.py on the robot model; returns (duration, reached_goal)."""
    ctrl = controller_class(**gains, ctrl_type=controller_type, log_dir=log_dir)
    pose_logger = Logger(os.path.join(log_dir, "robot_pose.csv"), ["x", "y", "theta", "stamp"])

    x, y, th = start
    v = w = 0.0
    pending = []          # commands waiting out the dead time: (time they act, v, w)
    v_cmd = w_cmd = 0.0   # command the robot is following
    steps = round(1 / (RATE * DT))

    for tick in range(int(max_time * RATE) + 1):
        t = tick / RATE
        stamp = round(t * 1e9)
        pose = [x, y, th, stamp]

        # same order as decisions.timerCallback: log the pose, stop at the goal, else ask the controller
        pose_logger.log_values([x, y, th, stamp])
        if calculate_linear_error(pose, goal) < tolerance:
            return t, True
        pending.append((t + DELAY, *ctrl.vel_request(pose, goal, True)))

        for i in range(steps):
            while pending and pending[0][0] <= t + i * DT + 1e-9:
                _, v_cmd, w_cmd = pending.pop(0)
            v_target = V_RATIO * v_cmd
            v_target = copysign(V_MIN, v_target) if 0 < abs(v_target) < V_MIN else v_target
            v += (v_target - v) * DT / TAU
            w += (W_RATIO * w_cmd - w) * DT / TAU
            x += v * cos(th) * DT
            y += v * sin(th) * DT
            th = atan2(sin(th + w * DT), cos(th + w * DT))   # odometry yaw is in [-pi, pi]

    return max_time, False


def main():
    parser = argparse.ArgumentParser(description="simulate a point run on a model of the robot")
    parser.add_argument("--like", required=True, help="robot run to copy the goal, gains, tolerance and start pose from")
    parser.add_argument("--controller", type=str.upper, default="PID", choices=list(CONTROLLER_TYPES))
    parser.add_argument("--improved", action="store_true", help="use pointController (controller.py) instead of controller")
    parser.add_argument("--gains-from", default=None, help="run to copy the gains from instead of the --like run")
    for gain in GAIN_NAMES:
        parser.add_argument(f"--{gain}", type=float, default=None, help=f"override the {gain} gain of the --like run")
    parser.add_argument("--goal-tolerance", type=float, default=None, help="override the goal tolerance of the --like run (m)")
    parser.add_argument("--max-time", type=float, default=60.0, help="stop if the goal is not reached by then (s)")
    parser.add_argument("--name", default="", help="optional label added to the run folder name")
    parser.add_argument("--runs-dir", default="sim_runs")
    args = parser.parse_args()

    with open(os.path.join(args.like, "params.json")) as file:
        like = json.load(file)
    base_gains = like["gains"]
    if args.gains_from:
        with open(os.path.join(args.gains_from, "params.json")) as file:
            base_gains = json.load(file)["gains"]
    gains = {name: getattr(args, name) if getattr(args, name) is not None else base_gains[name] for name in GAIN_NAMES}
    tolerance = args.goal_tolerance if args.goal_tolerance is not None else like["goal_tolerance"]
    first = pd.read_csv(os.path.join(args.like, "robot_pose.csv"), skipinitialspace=True).iloc[0]
    start = [float(first["x"]), float(first["y"]), float(first["theta"])]

    controller_class = pointController if args.improved else controller
    folder = f"sim_point_{args.controller}{'_improved' if args.improved else ''}_like_{os.path.basename(os.path.normpath(args.like))}"
    log_dir = os.path.join(args.runs_dir, f"{folder}_{args.name}" if args.name else folder)
    os.makedirs(log_dir, exist_ok=True)
    duration, reached = simulate(gains, CONTROLLER_TYPES[args.controller], like["goal"], start,
                                 tolerance, args.max_time, log_dir, controller_class)

    with open(os.path.join(log_dir, "params.json"), "w") as file:
        json.dump({"motion": "point",
                   "trajectory": None,
                   "goal": like["goal"],
                   "controller": args.controller,
                   "controller_class": controller_class.__name__,
                   "gains": gains,
                   "goal_tolerance": tolerance,
                   "name": args.name,
                   "started": datetime.now().isoformat(timespec="seconds"),
                   "source": "simulation (sim_point.py)",
                   "like_run": args.like,
                   "gains_from": args.gains_from or args.like,
                   "start_pose": start,
                   "model": {"delay_s": DELAY, "tau_s": TAU, "v_ratio": V_RATIO, "w_ratio": W_RATIO,
                             "v_min_mps": V_MIN, "rate_hz": RATE},
                   "reached_goal": reached,
                   "duration_s": duration}, file, indent=2)
    print(f"{args.controller}: {'reached the goal' if reached else 'did NOT reach the goal'} after {duration:.1f} s -> {log_dir}")


if __name__ == "__main__":
    main()
