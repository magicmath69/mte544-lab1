"""Render the recorded IMU channels used in the Lab 1 report.

Chart contract: three small-multiple raw time-series panels compare acceleration
and turn rate across circle, spiral, and line runs. Each CSV row is one message;
all available rows are plotted, without filtering, smoothing, or resampling.
Both quantities use common scales across runs and separate physical-unit axes.
Colors and line styles match the odometry figure. Outputs are a 7 x 2.15 inch
PDF, a 300 dpi PNG, and descriptive metrics beside this script.

Run from any directory: python analysis/imu_results.py
Dependencies: Python 3, NumPy, Matplotlib.
"""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MOTIONS = ("circle", "spiral", "line")
BLUE, GOLD, CHARCOAL = "#2458A6", "#B17A16", "#30343B"


def read_imu(path):
    """Keep the recorded order and subtract integer nanoseconds before casting."""
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        header = [field.strip() for field in next(reader)]
        if header[-1] == "":
            header.pop()
        if header != ["acc_x", "acc_y", "angular_z", "stamp"]:
            raise ValueError(f"Unexpected IMU header in {path.name}: {header}")
        values, stamps = [], []
        for row_number, row in enumerate(reader, start=2):
            fields = [field.strip() for field in row]
            if fields and fields[-1] == "":
                fields.pop()
            if len(fields) != 4:
                raise ValueError(f"{path.name}:{row_number}: expected four fields")
            values.append([float(value) for value in fields[:3]])
            stamps.append(int(fields[3]))
    signal = np.asarray(values, dtype=float)
    time = np.asarray([(stamp - stamps[0]) * 1e-9 for stamp in stamps])
    if not np.isfinite(signal).all() or not np.all(np.diff(time) > 0):
        raise ValueError(f"Non-finite data or non-increasing time in {path.name}")
    return time, signal


def channel_metrics(values):
    return {
        "minimum": float(values.min()),
        "maximum": float(values.max()),
        "mean": float(values.mean()),
        "median": float(np.median(values)),
        "population_standard_deviation": float(values.std()),
    }


def summarize(time, signal):
    intervals = np.diff(time)
    median_interval = float(np.median(intervals))
    result = {
        "samples": len(time),
        "duration_s": float(time[-1]),
        "timestamp_convention": "integer ns differences / 1e9; first sample is t=0",
        "sample_interval_min_median_max_s": [
            float(intervals.min()), median_interval, float(intervals.max())
        ],
        "non_increasing_intervals": int(np.count_nonzero(intervals <= 0)),
        "gaps_over_twice_median_interval": int(
            np.count_nonzero(intervals > 2 * median_interval)
        ),
        "non_finite_values": int(np.count_nonzero(~np.isfinite(signal))),
        "acc_x_m_s2": channel_metrics(signal[:, 0]),
        "acc_y_m_s2": channel_metrics(signal[:, 1]),
        "angular_z_rad_s": channel_metrics(signal[:, 2]),
        "interpretation": (
            "Full-run standard deviations describe observed variation, "
            "not isolated sensor noise or measurement accuracy."
        ),
    }
    windows = {}
    for start, end in ((0, 1), (2, 15), (5, 10), (15, 20), (25, 90)):
        if end > time[-1]:
            continue
        selected = signal[(time >= start) & (time < end)]
        windows[f"{start}_to_{end}_s"] = {
            "samples": len(selected),
            "angular_z_rad_s": channel_metrics(selected[:, 2]),
        }
    result["descriptive_time_windows"] = windows
    return result


def main():
    data = {
        motion: read_imu(ROOT / f"imu_content_{motion}.csv") for motion in MOTIONS
    }
    metrics = {motion: summarize(*data[motion]) for motion in MOTIONS}
    output = ROOT / "report" / "figures"
    output.mkdir(parents=True, exist_ok=True)
    (ROOT / "analysis" / "imu_metrics.json").write_text(
        json.dumps(metrics, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "mathtext.fontset": "dejavusans",
        "font.size": 8.5,
        "axes.titlesize": 9,
        "axes.labelsize": 8.5,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "axes.edgecolor": "#858A93",
        "axes.labelcolor": CHARCOAL,
        "text.color": CHARCOAL,
        "xtick.color": CHARCOAL,
        "ytick.color": CHARCOAL,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "path.simplify": False,
    })
    fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.15), sharey=True)
    fig.subplots_adjust(left=0.075, right=0.92, bottom=0.29, top=0.77, wspace=0.5)
    handles = None
    for axis, motion in zip(axes, MOTIONS):
        time, signal = data[motion]
        turn_axis = axis.twinx()
        ax_line, = axis.plot(time, signal[:, 0], color=BLUE, lw=0.6,
                             label=r"$a_x$ (left)")
        ay_line, = axis.plot(time, signal[:, 1], color=GOLD, ls="--", lw=0.7,
                             label=r"$a_y$ (left)")
        wz_line, = turn_axis.plot(time, signal[:, 2], color=CHARCOAL, ls=":",
                                  lw=1.0, label=r"$\omega_z$ (right)")
        handles = (ax_line, ay_line, wz_line)
        axis.set_title(
            f"{motion.capitalize()}\nn = {len(time):,}; T = {time[-1]:.2f} s",
            pad=5, linespacing=1.2,
        )
        axis.set_xlim(0, time[-1])
        axis.set_ylim(-1.3, 1.6)
        axis.set_yticks((-1, 0, 1))
        turn_axis.set_ylim(-0.1, 0.8)
        turn_axis.set_yticks((0, 0.4, 0.8))
        axis.xaxis.set_major_locator(MaxNLocator(nbins=3))
        axis.set_xlabel("Elapsed time (s)", labelpad=3)
        axis.grid(color="#D8DBE1", lw=0.6)
        axis.set_axisbelow(True)
        axis.spines["top"].set_visible(False)
        turn_axis.spines["top"].set_visible(False)
        axis.tick_params(axis="both", length=3, pad=2)
        turn_axis.tick_params(axis="y", length=3, pad=2)
    axes[0].set_ylabel(r"$a_x,a_y$ (m/s$^2$)", labelpad=4)
    fig.axes[-1].set_ylabel(r"$\omega_z$ (rad/s)", labelpad=4)
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, 0.005), handlelength=2.7,
               columnspacing=1.6, handletextpad=0.6)
    fig.savefig(output / "imu_results.pdf", metadata={"Title": "Raw IMU results"})
    fig.savefig(output / "imu_results.png", dpi=300)
    plt.close(fig)
    print(json.dumps({motion: {
        "samples": values["samples"], "duration_s": values["duration_s"],
        "mean_angular_z_rad_s": values["angular_z_rad_s"]["mean"],
    } for motion, values in metrics.items()}, indent=2))


if __name__ == "__main__":
    main()
