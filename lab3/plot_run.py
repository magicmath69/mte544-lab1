# Claude: plots one run's log to compare the EKF with the odometry (and the IMU) in the lab.
# Claude: usage:  python3 plot_run.py                     newest log in runs/
# Claude:         python3 plot_run.py runs/<file>.csv     that log (give several files for one figure each)
import sys
import os
import glob

import numpy as np
import matplotlib.pyplot as plt

RUNS_DIR="runs"


# Claude: reads a Logger csv (lines like "a, b, c, ") into {column name: numpy array}
def read_log(path):
    with open(path) as file:
        rows=[[cell.strip() for cell in line.split(",") if cell.strip()] for line in file if line.strip()]
    data=np.array(rows[1:], dtype=float).reshape(-1, len(rows[0]))    # Claude: reshape so a log with no data rows (run stopped early) stays 2-D
    return {name: data[:, i] for i, name in enumerate(rows[0])}


def plot_run(path):
    log=read_log(path)
    if len(log["stamp"]) < 2:
        print(f"{path}: fewer than 2 rows, nothing to plot")
        return

    t=(log["stamp"] - log["stamp"][0]) / 1e9

    # Claude: the EKF always starts at (0, 0, 0). Odometry is moved into its own start frame so it starts there too,
    # Claude: which makes the comparison right even if the odometry wasn't reset before the run.
    x0, y0, th0=log["odom_x"][0], log["odom_y"][0], log["odom_th"][0]
    dx, dy=log["odom_x"] - x0, log["odom_y"] - y0
    odom_x=np.cos(th0)*dx + np.sin(th0)*dy
    odom_y=-np.sin(th0)*dx + np.cos(th0)*dy
    odom_th=np.unwrap(log["odom_th"]) - th0
    gap=np.hypot(log["kf_x"] - odom_x, log["kf_y"] - odom_y)

    ekf=dict(color="tab:red", linestyle="--", linewidth=2, label="EKF")
    odom=dict(color="tab:blue", linestyle="-", linewidth=1.2, label="odometry")
    imu=dict(color="tab:gray", linestyle="-", linewidth=0.8, alpha=0.7, label="IMU")

    fig, axes=plt.subplots(2, 3, figsize=(15, 8))
    name=os.path.basename(path)
    fig.suptitle(f"{name}: {t[-1]:.1f} s, EKF vs odometry position gap: final {gap[-1]*100:.1f} cm, max {gap.max()*100:.1f} cm")

    ax=axes[0, 0]
    ax.plot(odom_x, odom_y, **odom)
    ax.plot(log["kf_x"], log["kf_y"], **ekf)
    ax.plot(0, 0, "ko", label="start")
    ax.set(title="Path", xlabel="x (m)", ylabel="y (m)")
    ax.axis("equal")

    panels=[(axes[0, 1], "Heading", "θ (rad)", (odom_th, odom), log["kf_th"]),
            (axes[0, 2], "Linear velocity", "v (m/s)", (log["odom_v"], odom), log["kf_vx"]),
            (axes[1, 0], "Angular velocity", "ω (rad/s)", (log["odom_w"], odom), log["kf_w"]),
            (axes[1, 1], "Forward acceleration", "a_x (m/s²)", (log["imu_ax"], imu), log["kf_ax"]),
            (axes[1, 2], "Sideways acceleration", "a_y (m/s²)", (log["imu_ay"], imu), log["kf_ay"])]
    for ax, title, ylabel, (measured, style), estimate in panels:
        ax.plot(t, measured, **style)
        ax.plot(t, estimate, **ekf)
        ax.set(title=title, xlabel="time (s)", ylabel=ylabel)

    for ax in axes.flat:
        ax.grid(True)
        ax.legend()

    fig.tight_layout()
    print(f"{path}: {len(t)} rows, {t[-1]:.1f} s, final gap {gap[-1]*100:.1f} cm, max gap {gap.max()*100:.1f} cm")


if __name__=="__main__":
    files=sys.argv[1:]
    if not files:
        logs=glob.glob(os.path.join(RUNS_DIR, "*.csv"))
        if not logs:
            sys.exit(f"no logs in {RUNS_DIR}/")
        files=[max(logs, key=os.path.getmtime)]
    for path in files:
        plot_run(path)
    plt.show()
