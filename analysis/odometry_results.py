"""Regenerate odometry evidence from the original 2,367 CSV poses.

Chart contract: 7 x 3.15 inch PDF/PNG with x-y paths above raw position/yaw
histories. Every time-history panel labels metres left and radians right;
retain the blue/gold/charcoal palette, line styles, and original data scales.
"""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator, ScalarFormatter
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BLUE, GOLD, INK = "#2458A6", "#B17A16", "#30343B"


def read_motion(motion):
    with (ROOT / f"odom_content_{motion}.csv").open(newline="") as stream:
        reader = csv.reader(stream)
        assert [v.strip() for v in next(reader) if v.strip()] == ["x", "y", "th", "stamp"]
        rows = [[v.strip() for v in row if v.strip()] for row in reader if any(v.strip() for v in row)]
    assert all(len(row) == 4 for row in rows)
    pose = np.array([[float(v) for v in row[:3]] for row in rows])
    stamps = np.array([int(row[3]) for row in rows], dtype=np.int64)
    assert np.isfinite(pose).all() and np.all(np.diff(stamps) > 0)
    return (stamps - stamps[0]) / 1e9, pose, np.unwrap(pose[:, 2])


def main():
    motions = ("circle", "spiral", "line")
    data = {name: read_motion(name) for name in motions}
    metrics = {}
    for name, (time, pose, yaw) in data.items():
        metrics[name] = {
            "samples": len(time), "duration_s": float(time[-1]),
            "sample_interval_min_median_max_s": np.quantile(np.diff(time), [0, .5, 1]).tolist(),
            "path_length_m": float(np.linalg.norm(np.diff(pose[:, :2], axis=0), axis=1).sum()),
            "endpoint_displacement_m": float(np.linalg.norm(pose[-1, :2] - pose[0, :2])),
            "unwrapped_heading_change_rad": float(yaw[-1] - yaw[0]),
            "heading_span_rad": float(np.ptp(yaw)),
            "start_pose": pose[0].tolist(), "end_pose": pose[-1].tolist(),
            "yaw_wraps": int(np.sum(np.abs(np.diff(pose[:, 2])) > np.pi)),
        }
    circle = data["circle"][1][:, :2]
    design = np.column_stack((2 * circle, np.ones(len(circle))))
    cx, cy, c = np.linalg.lstsq(design, np.sum(circle ** 2, axis=1), rcond=None)[0]
    metrics["circle"]["fitted_radius_m"] = float(np.sqrt(c + cx ** 2 + cy ** 2))
    metrics["circle"]["completed_turns"] = metrics["circle"]["unwrapped_heading_change_rad"] / (2 * np.pi)
    spiral_time, spiral_pose, _ = data["spiral"]
    late_mask = spiral_time >= 60
    late_xy = spiral_pose[late_mask, :2]
    cx, cy, c = np.linalg.lstsq(
        np.column_stack((2 * late_xy, np.ones(len(late_xy)))),
        np.sum(late_xy ** 2, axis=1), rcond=None,
    )[0]
    outer_radius = float(np.sqrt(c + cx ** 2 + cy ** 2))
    metrics["spiral"]["outer_loop_fit"] = {
        "start_s": float(spiral_time[late_mask][0]),
        "end_s": float(spiral_time[-1]), "samples": len(late_xy),
        "radius_m": outer_radius,
        "radial_rmse_m": float(np.sqrt(np.mean((np.linalg.norm(late_xy - [cx, cy], axis=1) - outer_radius) ** 2))),
    }
    (ROOT / "analysis" / "odometry_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    sections = ROOT / "report" / "sections"
    sections.mkdir(parents=True, exist_ok=True)
    values = {
        "odomCircleRadius": f"{metrics['circle']['fitted_radius_m']:.3f}",
        "odomCircleTurns": f"{metrics['circle']['completed_turns']:.3f}",
        "odomCircleYaw": f"{metrics['circle']['unwrapped_heading_change_rad']:.2f}",
        "odomCircleDuration": f"{metrics['circle']['duration_s']:.2f}",
        "odomSpiralRadius": f"{outer_radius:.3f}",
        "odomSpiralYaw": f"{metrics['spiral']['unwrapped_heading_change_rad']:.2f}",
        "odomSpiralDuration": f"{metrics['spiral']['duration_s']:.2f}",
        "odomLineLength": f"{metrics['line']['path_length_m']:.3f}",
        "odomLineDuration": f"{metrics['line']['duration_s']:.2f}",
        "odomLineYawSpan": f"{metrics['line']['heading_span_rad']:.4f}",
        "odomLineYawDegrees": f"{np.degrees(metrics['line']['heading_span_rad']):.2f}",
    }
    (sections / "odometry_numbers.tex").write_text(
        "".join(f"\\newcommand{{\\{key}}}{{{value}}}\n" for key, value in values.items())
    )
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 9,
        "axes.titlesize": 9, "axes.labelsize": 8.5,
        "xtick.labelsize": 8, "ytick.labelsize": 8,
        "text.color": INK, "axes.labelcolor": INK,
        "axes.edgecolor": "#838891", "axes.linewidth": .65,
        "grid.color": "#DBDEE3", "grid.linewidth": .5, "pdf.fonttype": 42,
    })
    fig, axes = plt.subplots(2, 3, figsize=(7, 3.15))
    fig.subplots_adjust(left=.07, right=.9, bottom=.18, top=.91, wspace=.80, hspace=.72)
    for column, name in enumerate(motions):
        time, pose, yaw = data[name]
        top, bottom = axes[:, column]
        top.plot(pose[:, 0], pose[:, 1], color=BLUE, linewidth=.9)
        top.plot(*pose[0, :2], "o", ms=4.4, mfc="white", mec=INK, zorder=4)
        top.plot(*pose[-1, :2], "^", ms=4.4, color=INK, zorder=5)
        span = np.ptp(pose[:, :2], axis=0).max() * 1.24
        centre = (pose[:, :2].min(axis=0) + pose[:, :2].max(axis=0)) / 2
        top.set(xlim=(centre[0] - span / 2, centre[0] + span / 2),
                ylim=(centre[1] - span / 2, centre[1] + span / 2), xlabel="x (m)", ylabel="y (m)")
        top.set_aspect("equal", adjustable="box")
        top.set_title(f"{name.capitalize()}: trajectory", loc="left", pad=4)
        bottom.plot(time, pose[:, 0], color=BLUE, lw=1)
        bottom.plot(time, pose[:, 1], color=GOLD, lw=1, ls="--")
        right = bottom.twinx()
        right.plot(time, yaw, color=INK, lw=1, ls=":")
        right.yaxis.set_major_locator(MaxNLocator(3))
        right.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
        right.spines["top"].set_visible(False)
        right.set_ylabel(r"$\theta$ (rad)", labelpad=2)
        bottom.set_ylabel("x, y (m)", labelpad=2)
        bottom.set(xlabel="Elapsed time (s)", xlim=(0, time[-1]))
        bottom.set_title(f"n={len(time):,}; T={time[-1]:.2f} s", fontsize=8, loc="left", pad=4)
        for ax in (top, bottom):
            ax.grid(True)
            ax.set_axisbelow(True)
            ax.xaxis.set_major_locator(MaxNLocator(3))
            ax.yaxis.set_major_locator(MaxNLocator(3))
            ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)
            ax.xaxis.labelpad = ax.yaxis.labelpad = 2
    handles = [
        Line2D([], [], marker="o", mfc="white", color=INK, ls="none", ms=4, label="Start"),
        Line2D([], [], marker="^", color=INK, ls="none", ms=4, label="End"),
        Line2D([], [], color=BLUE, lw=1, label="x"),
        Line2D([], [], color=GOLD, lw=1, ls="--", label="y"),
        Line2D([], [], color=INK, lw=1, ls=":", label=r"$\theta$ (unwrapped)"),
    ]
    fig.legend(handles=handles, ncol=5, frameon=False, fontsize=8.5,
               loc="lower center", bbox_to_anchor=(.5, -.005), columnspacing=1.2)
    figures = ROOT / "report" / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    for suffix in ("pdf", "png"):
        fig.savefig(figures / f"odometry_results.{suffix}", dpi=300, facecolor="white")
    plt.close(fig)
    print("Odometry: 2,367 valid poses; circle radius", values["odomCircleRadius"], "m; line", values["odomLineLength"], "m")


if __name__ == "__main__":
    main()
