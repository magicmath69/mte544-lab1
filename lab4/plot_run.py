# plots a run on the map: planned path(s), EKF and odometry poses, start and goal(s), plus the controller errors
# usage: python3 plot_run.py [runs/<run>_pose.csv ...] [--map room]   (default: newest run in runs/)
import sys
import os
import glob
import argparse

import numpy as np
import matplotlib.pyplot as plt

RUNS_DIR="runs"


# Logger csv -> {column name: array}
def read_log(path):
    with open(path) as file:
        rows=[[cell.strip() for cell in line.split(",") if cell.strip()] for line in file if line.strip()]
    data=np.array(rows[1:], dtype=float).reshape(-1, len(rows[0]))
    return {name: data[:, i] for i, name in enumerate(rows[0])}


# map.yaml + map.pgm -> image and its extent in m (no ROS needed, so this also works off the lab PC)
def read_map(name):
    meta={}
    with open(name + ".yaml") as file:
        for line in file:
            if ":" in line:
                key, value=line.split(":", 1)
                meta[key.strip()]=value.strip()
    res=float(meta["resolution"])
    ox, oy=[float(v) for v in meta["origin"].strip("[]").split(",")[:2]]

    with open(name + ".pgm", "rb") as file:
        fields=[]
        while len(fields) < 4:   # P5, width, height, max value (skipping comments)
            line=file.readline()
            if not line.startswith(b"#"):
                fields+=line.split()
        width, height=int(fields[1]), int(fields[2])
        image=np.frombuffer(file.read(width*height), dtype=np.uint8).reshape(height, width)
    return image, [ox, ox + width*res, oy, oy + height*res]


def plot_run(pose_path, map_name):
    run=pose_path[:-len("_pose.csv")]
    pose=read_log(pose_path)
    path=read_log(run + "_path.csv") if os.path.exists(run + "_path.csv") else None

    fig, axes=plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [3, 2]})
    fig.suptitle(os.path.basename(run))

    ax=axes[0]
    if os.path.exists(map_name + ".yaml"):
        image, extent=read_map(map_name)
        ax.imshow(image, cmap="gray", extent=extent)
    else:
        print(f"no {map_name}.yaml here, plotting without the map")

    if path is not None and len(path["goal"]):
        for i, goal in enumerate(np.unique(path["goal"])):
            points=path["goal"] == goal
            color=["tab:orange", "tab:green", "tab:cyan", "tab:olive"][i % 4]
            ax.plot(path["x"][points], path["y"][points], "--", color=color, linewidth=1.5, label=f"planned path {int(goal)}")
            ax.plot(path["x"][points][-1], path["y"][points][-1], "x", color=color, markersize=10, markeredgewidth=2, label=f"goal {int(goal)}")
    ax.plot(pose["odom_x"], pose["odom_y"], color="tab:blue", linewidth=1.2, label="odometry")
    if not np.all(np.isnan(pose["kf_x"])):
        ax.plot(pose["kf_x"], pose["kf_y"], color="tab:red", linewidth=1.2, label="EKF")
    if len(pose["odom_x"]):
        ax.plot(pose["odom_x"][0], pose["odom_y"][0], "go", label="start")
    ax.set(title="Path on map", xlabel="x (m)", ylabel="y (m)")
    ax.axis("equal")

    # controller error logs from the course PID code (e, e_dot, e_int, stamp)
    ax=axes[1]
    for suffix, label in [("_linear.csv", "distance to goal (m)"), ("_angular.csv", "heading error (rad)")]:
        if os.path.exists(run + suffix):
            errors=read_log(run + suffix)
            if len(errors["stamp"]):
                ax.plot((errors["stamp"] - errors["stamp"][0]) / 1e9, errors["e"], label=label)
    ax.set(title="Controller errors", xlabel="time (s)", ylabel="error")

    for ax in axes:
        ax.grid(True)
        ax.legend(fontsize=8)
    fig.tight_layout()

    if len(pose["stamp"]) > 1:
        print(f"{run}: {len(pose['stamp'])} pose rows, {(pose['stamp'][-1] - pose['stamp'][0]) / 1e9:.1f} s")


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("files", nargs="*", help="runs/<run>_pose.csv (default: newest run)")
    parser.add_argument("--map", type=str, default="room", help="map name without extension")
    args=parser.parse_args()

    files=args.files
    if not files:
        logs=glob.glob(os.path.join(RUNS_DIR, "*_pose.csv"))
        if not logs:
            sys.exit(f"no logs in {RUNS_DIR}/")
        files=[max(logs, key=os.path.getmtime)]
    for path in files:
        plot_run(path, args.map)
    plt.show()
