"""Generate Lab 4 report figures and results from the saved map and run logs.

From lab4/: python report/make_figures.py runs report/figures --map room
Requires matplotlib, NumPy, pandas and SciPy. No ROS environment is needed.
"""

import argparse
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.ndimage import distance_transform_edt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from a_star import search


GOAL_RADIUS_M = 0.05
BLOCKED_COST = 0.8
PLAN = "#b84c0c"
EKF = "#1757a5"
ODOM = "#393838"
START = "#246139"
MANHATTAN = "#148369"


def load_log(filename):
    log = pd.read_csv(filename, skipinitialspace=True)
    return log.rename(columns=str.strip)


def read_map(name):
    lines = name.with_suffix(".yaml").read_text().splitlines()
    meta = dict(line.split(":", 1) for line in lines if ":" in line)
    resolution = float(meta["resolution"])
    origin = np.array(json.loads(meta["origin"])[:2])
    with name.with_suffix(".pgm").open("rb") as pgm:
        header = []
        while len(header) < 4:
            line = pgm.readline()
            if not line:
                raise ValueError(f"Incomplete PGM header for {name}")
            header.extend(line.partition(b"#")[0].split())
        width, height = map(int, header[1:3])
        image = np.frombuffer(pgm.read(width * height), dtype=np.uint8).reshape(height, width)

    # mapUtilities.cell_2_position assigns these coordinates to cell centers.
    # Adjust the image edges by half a pixel to align the paths and map.
    x_left, y_bottom = origin + resolution * np.array([-0.5, 0.5])
    extent = [x_left, x_left + width * resolution, y_bottom, y_bottom + height * resolution]
    return image, resolution, origin, extent


def plot_map(ax, image, extent, title, xlim=None, ylim=None, **image_options):
    raster = ax.imshow(image, extent=extent, interpolation="nearest", **image_options)
    ax.set(title=title, xlabel="x [m]", ylabel="y [m]", xlim=xlim, ylim=ylim)
    ax.grid(color="#727575", alpha=0.28, linewidth=0.45)
    return raster


def plot_figure(fig, filename):
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def distance_to_path(points, route):
    """Measure distance to the closest point on the path polyline."""
    steps = np.diff(route, axis=0)
    relative = points[:, None, :] - route[None, :-1, :]
    projection = np.clip(np.einsum("nsi,si->ns", relative, steps) / (steps**2).sum(axis=1), 0, 1)
    return np.linalg.norm(relative - projection[..., None] * steps, axis=2).min(axis=1)


def measure_run(pose_file):
    name = pose_file.name.removesuffix("_pose.csv")
    path_log = load_log(pose_file.with_name(f"{name}_path.csv"))
    if path_log["goal"].nunique() != 1:
        raise ValueError(f"Expected exactly one logged goal for {name}")

    route = path_log[["x", "y"]].to_numpy()
    if len(route) < 2:
        raise ValueError(f"Expected at least two planned waypoints in {name}")

    pose = load_log(pose_file)
    linear = load_log(pose_file.with_name(f"{name}_linear.csv"))
    command_stamp = linear["stamp"].iloc[0]
    active = pose.loc[pose["stamp"] >= command_stamp]
    ekf = active[["kf_x", "kf_y"]].to_numpy()
    arrival = np.flatnonzero(np.linalg.norm(ekf - route[-1], axis=1) < GOAL_RADIUS_M)
    if not arrival.size:
        raise ValueError(f"No EKF sample reached the goal in {name}")

    active = active.iloc[:arrival[0] + 1]
    ekf = active[["kf_x", "kf_y"]].to_numpy()
    odom = active[["odom_x", "odom_y"]].to_numpy()

    steps = np.diff(route, axis=0)
    ekf_cross_track = distance_to_path(ekf, route)
    odom_cross_track = distance_to_path(odom, route)

    return route, ekf, odom, {
        "time_to_goal_s": round(float(active["stamp"].iloc[-1] - command_stamp) / 1e9, 2),
        "path_len_m": round(float(np.linalg.norm(steps, axis=1).sum()), 2),
        "path_cells": len(route),
        "start": route[0].round(2).tolist(),
        "goal": route[-1].round(2).tolist(),
        "final_ekf_position": ekf[-1].round(4).tolist(),
        "final_odom_to_goal_m": round(float(np.linalg.norm(odom[-1] - route[-1])), 4),
        "final_ekf_to_goal_m": round(float(np.linalg.norm(ekf[-1] - route[-1])), 4),
        "max_ekf_odom_gap_m": round(float(np.linalg.norm(ekf - odom, axis=1).max()), 4),
        "mean_ekf_cross_track_m": round(float(ekf_cross_track.mean()), 4),
        "mean_odom_cross_track_m": round(float(odom_cross_track.mean()), 4),
        "rms_ekf_cross_track_m": round(float(np.sqrt(np.mean(ekf_cross_track**2))), 4),
        "max_ekf_cross_track_m": round(float(ekf_cross_track.max()), 4),
    }


def gaussian_cost(clearance, sigma):
    return np.exp(-0.5 * (clearance / sigma)**2)


def replan(route, clearance, cost, origin, resolution, heuristic):
    # The logged endpoints are already cell centers. Round after converting
    # back to indices to avoid assigning a boundary to its adjacent cell.
    height = clearance.shape[0]
    endpoints = np.rint((route[[0, -1]] - origin) / resolution).astype(int)
    endpoints[:, 1] = height - endpoints[:, 1]
    cells = search(cost, *endpoints, heuristic)
    if cells is None:
        raise ValueError(f"{heuristic} found no path between the logged endpoints")

    x, y = np.array(cells).T
    xy = origin + resolution * np.column_stack((x, height - y))
    steps = np.hypot(np.diff(x), np.diff(y))
    wall_distance = clearance[y, x]
    return xy, {
        "path_len_m": round(float(steps.sum() * resolution), 2),
        "cells": len(cells),
        "path_cost": round(float(steps.sum() + cost[y[1:], x[1:]].sum()), 3),
        "min_wall_clearance_m": round(float(wall_distance.min()), 3),
        "mean_wall_clearance_m": round(float(wall_distance.mean()), 3),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--map", type=Path, default=Path("room"))
    parser.add_argument("--laser_sig", type=float, default=0.4)
    parser.add_argument("--keep", nargs="+", default=["165419", "170619"])
    parser.add_argument("--xlim", type=float, nargs=2, default=[-1.5, 7.5])
    parser.add_argument("--ylim", type=float, nargs=2, default=[-2.5, 3.5])
    args = parser.parse_args()
    if args.laser_sig <= 0:
        parser.error("--laser_sig must be positive")

    plt.rcParams.update({
        "font.size": 8,
        "axes.titlesize": 8.5,
        "axes.labelsize": 8,
        "axes.linewidth": 0.65,
        "axes.axisbelow": True,
        "legend.fontsize": 7,
        "legend.frameon": False,
        "pdf.fonttype": 42,
    })
    args.out.mkdir(parents=True, exist_ok=True)

    image, resolution, origin, extent = read_map(args.map)
    clearance = distance_transform_edt(image >= 10) * resolution
    cost = gaussian_cost(clearance, args.laser_sig)
    baseline_sigma = 0.3
    baseline_cost = gaussian_cost(clearance, baseline_sigma)
    numbers = {
        "laser_sig": args.laser_sig,
        "blocked_radius_m": round(float(np.sqrt(-2 * np.log(BLOCKED_COST)) * args.laser_sig), 3),
        "goal_tolerance_m": GOAL_RADIUS_M,
        "comparison_laser_sig": baseline_sigma,
    }

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.45), layout="constrained")
    plot_map(axes[0], image, extent, "(a) SLAM occupancy map", cmap="gray", vmin=0, vmax=254)
    raster = plot_map(axes[1], cost, extent, rf"(b) A* cost map ($\sigma={args.laser_sig:g}$ m)",
                      cmap="magma_r", vmin=0, vmax=1)
    axes[1].contour(cost, levels=[BLOCKED_COST], colors=["#1398cc"], linewidths=0.55,
                    origin="upper", extent=extent)
    fig.colorbar(raster, ax=axes[1], label="Cell cost", shrink=0.86, pad=0.025)
    plot_figure(fig, args.out / "fig_map.pdf")

    count = len(args.keep)
    run_fig, (run_axes,) = plt.subplots(1, count, figsize=(7.1, 2.85), squeeze=False)
    plan_fig, (plan_axes,) = plt.subplots(1, count, figsize=(7.1, 2.85), squeeze=False)
    for index, (run_id, run_ax, plan_ax) in enumerate(zip(args.keep, run_axes, plan_axes)):
        candidates = list(args.runs.glob(f"*{run_id}*_pose.csv"))
        if len(candidates) != 1:
            raise ValueError(f"Expected one pose log for {run_id}, found {len(candidates)}")
        pose_file = candidates[0]
        name = pose_file.name.removesuffix("_pose.csv")
        route, ekf, odom, metrics = measure_run(pose_file)
        numbers[name] = metrics

        panel = chr(ord("a") + index)
        title = f"({panel}) Run {run_id}"
        plot_map(run_ax, image, extent, title, args.xlim, args.ylim, cmap="gray", vmin=0, vmax=254)
        run_ax.plot(*route.T, "--", color=PLAN, lw=1.15, label="Planned path")
        run_ax.plot(*odom.T, color=ODOM, lw=0.85, label="Odometry")
        run_ax.plot(*ekf.T, color=EKF, lw=1.1, label="EKF trajectory")
        run_ax.plot(*ekf[0], "o", color=START, ms=4, label="Tracking start")
        run_ax.plot(*ekf[-1], "o", mfc="none", mec=EKF, ms=7, label="EKF end")
        run_ax.plot(*route[-1], "x", color=PLAN, ms=4.5, label="Planned goal")

        plot_map(plan_ax, image, extent, title, args.xlim, args.ylim, cmap="gray", vmin=0, vmax=254)
        comparisons = {}
        for heuristic, color, style in (("euclidean", PLAN, "-"), ("manhattan", MANHATTAN, "--")):
            xy, planned = replan(route, clearance, cost, origin, resolution, heuristic)
            planned["time_to_completion_s"] = metrics["time_to_goal_s"] if heuristic == "euclidean" else None
            comparisons[heuristic] = planned
            if heuristic == "euclidean":
                comparisons["matches_logged_path"] = bool(route.shape == xy.shape and np.allclose(route, xy))

            _, baseline = replan(route, clearance, baseline_cost, origin, resolution, heuristic)
            planned["comparison_min_wall_clearance_m"] = baseline["min_wall_clearance_m"]
            plan_ax.plot(*xy.T, style, color=color, lw=1.25, label=heuristic.capitalize())

        numbers[f"heuristics_{run_id}"] = comparisons
        plan_ax.plot(*route[0], "o", color=START, ms=4, label="Planned start")
        plan_ax.plot(*route[-1], "x", color=ODOM, ms=5, label="Planned goal")

    for fig, axes, ncol, filename in (
        (run_fig, run_axes, 3, "fig_runs.pdf"),
        (plan_fig, plan_axes, 4, "fig_heuristics.pdf"),
    ):
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="lower center", ncol=ncol)
        fig.tight_layout(rect=(0, 0.14, 1, 1), w_pad=1.35)
        plot_figure(fig, args.out / filename)

    (args.out / "numbers.json").write_text(json.dumps(numbers, indent=2) + "\n")


if __name__ == "__main__":
    main()
