"""Reproduce the report's LiDAR snapshots and saved occupancy-map figure.

This analysis helper reads recorded files; it does not control the robot.
Run from any directory: python analysis/lidar_mapping_results.py
"""

import ast
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
FIGURES = ROOT / "report" / "figures"
INK, BLUE, GRID = "#30343B", "#2458A6", "#E4E6E9"
plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 8.5,
    "text.color": INK, "axes.labelcolor": INK, "axes.edgecolor": INK,
    "xtick.color": INK, "ytick.color": INK, "axes.titleweight": "normal",
    "axes.titlesize": 9, "axes.linewidth": 0.6,
    "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "xtick.major.size": 2.5, "ytick.major.size": 2.5,
    "pdf.fonttype": 42, "savefig.facecolor": "white",
})


def load_scans(path):
    """Rows contain N ranges followed by angle_min, increment, and integer ns."""
    scans = []
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        next(reader)  # Four header names do not describe the flattened ranges.
        for row in reader:
            fields = [field.strip() for field in row if field.strip()]
            if fields:
                scans.append((np.array(fields[:-3], dtype=float),
                              float(fields[-3]), float(fields[-2]),
                              int(fields[-1])))
    return scans


def range_counts(ranges):
    return {
        "beams": int(ranges.size),
        "finite_positive": int(np.count_nonzero(np.isfinite(ranges) & (ranges > 0))),
        "positive_infinity": int(np.count_nonzero(np.isposinf(ranges))),
        "negative_infinity": int(np.count_nonzero(np.isneginf(ranges))),
        "nan": int(np.count_nonzero(np.isnan(ranges))),
        "nonpositive_finite": int(np.count_nonzero(np.isfinite(ranges) & (ranges <= 0))),
    }


def save_figure(fig, stem):
    fig.savefig(FIGURES / f"{stem}.pdf")
    fig.savefig(FIGURES / f"{stem}.png", dpi=300)
    plt.close(fig)


def lidar_results():
    # Contract: three 456--488-point Cartesian scatter snapshots compare local
    # sensed geometry at the nearest temporal midpoint of each recorded motion.
    # Static report export: 7 x 2 in, common scales, equal aspect, blue points;
    # black plus identifies the sensor origin without relying on color alone.
    chosen, metrics = [], {}
    for motion in ("circle", "spiral", "line"):
        scans = load_scans(ROOT / f"laser_content_{motion}.csv")
        first, last = scans[0][3], scans[-1][3]
        # Integer arithmetic preserves nanosecond differences and chooses the
        # earliest row if two scans have identical midpoint distance.
        index = min(range(len(scans)), key=lambda i: abs(2 * scans[i][3] - first - last))
        ranges, angle_min, increment, stamp = scans[index]
        beam_indices = np.arange(ranges.size)
        angles = angle_min + beam_indices * increment
        valid = np.isfinite(ranges) & (ranges > 0)
        # Apply the mask only after deriving angles from original beam indices.
        x, y = ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])
        elapsed = (stamp - first) * 1e-9
        chosen.append((motion, elapsed, x, y))
        metrics[motion] = {
            "source": f"laser_content_{motion}.csv", "scan_count": len(scans),
            "beam_counts": sorted({int(s[0].size) for s in scans}),
            "duration_s": (last - first) * 1e-9,
            "selected_row_zero_based": index,
            "selected_stamp_ns": stamp, "selected_elapsed_s": elapsed,
            "selected_midpoint_error_s": abs(2 * stamp - first - last) * 0.5e-9,
            "angle_min_rad": angle_min, "angle_increment_rad": increment,
            "selected_ranges": range_counts(ranges),
            "all_ranges": range_counts(np.concatenate([s[0] for s in scans])),
            "selected_finite_positive_range_m": [float(ranges[valid].min()), float(ranges[valid].max())],
            "selected_xy_bounds_m": {"x": [float(x.min()), float(x.max())],
                                     "y": [float(y.min()), float(y.max())]},
        }

    # One square domain encompasses every selected endpoint without clipping.
    bound = float(5 * np.ceil(max(np.max(np.abs(v)) for _, _, x, y in chosen for v in (x, y)) / 5))
    fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.0), sharex=True, sharey=True)
    fig.subplots_adjust(left=0.06, right=0.99, bottom=0.30, top=0.83, wspace=0.20)
    for ax, (motion, elapsed, x, y) in zip(axes, chosen):
        ax.scatter(x, y, s=2.4, color=BLUE, linewidths=0, zorder=3)
        ax.scatter([0], [0], s=25, marker="+", color=INK, linewidths=0.8, zorder=4)
        ax.set(title=f"{motion.capitalize()} ({elapsed:.2f} s)", xlabel="x (m)",
               xlim=(-bound, bound), ylim=(-bound, bound))
        ax.set_aspect("equal", adjustable="box")
        ax.set_xticks([-10, 0, 10])
        ax.set_yticks([-10, 0, 10])
        ax.grid(color=GRID, linewidth=0.45, zorder=0)
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(labelsize=8.5, pad=2)
    axes[0].set_ylabel("y (m)", labelpad=2)
    fig.legend(handles=[
        Line2D([], [], color=BLUE, marker="o", linestyle="none", markersize=3,
               label="Finite returns"),
        Line2D([], [], color=INK, marker="+", linestyle="none", markersize=5,
               label="Sensor origin"),
    ], loc="lower center", ncol=2, frameon=False, fontsize=8.5,
        handlelength=0.8, handletextpad=0.4, columnspacing=1.8,
        borderaxespad=0.1, bbox_to_anchor=(0.51, 0.0))
    save_figure(fig, "lidar_results")
    return {
        "selection": "Nearest scan to the midpoint of first and last logged timestamps; earliest row wins ties.",
        "range_filter": "Keep finite strictly positive ranges. Sensor range_min/range_max were not logged; no undocumented range bounds imposed.",
        "coordinates": "alpha_i = angle_min + original_beam_index_i * angle_increment; x_i = r_i cos(alpha_i), y_i = r_i sin(alpha_i). Sensor frame; no odometry transform.",
        "common_plot_domain_m": [-bound, bound], "motions": metrics,
    }


def map_results():
    # Contract: faithful nearest-neighbor PGM raster with metric map-frame axes;
    # black occupied, light-gray unknown, white free; no inferred trajectory.
    metadata = {}
    for line in (ROOT / "map.yaml").read_text().splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            metadata[key.strip()] = value.strip()
    resolution = float(metadata["resolution"])
    origin = ast.literal_eval(metadata["origin"])
    assert origin[2] == 0, "This saved map has zero yaw; rotated origins require a transform."
    pixels = np.asarray(Image.open(ROOT / metadata["image"]))
    height, width = pixels.shape
    xmin, ymin = origin[:2]
    xmax, ymax = xmin + width * resolution, ymin + height * resolution
    assert metadata["mode"] == "trinary" and int(metadata["negate"]) == 0
    probability = (255.0 - pixels) / 255.0
    occupied = probability > float(metadata["occupied_thresh"])
    free = probability < float(metadata["free_thresh"])
    unknown = ~(occupied | free)
    # Humble map saver writes 0/254/205 for occupied/free/unknown (or
    # intermediate) cells. Its loader instead applies the YAML thresholds to
    # 1-gray/255, so this file's free_thresh=.25 reloads gray205 as free.
    # Preserve the image convention in the plot and record both interpretations.
    # Source: https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp
    image_masks = {"occupied": pixels == 0, "free": pixels == 254, "unknown_or_intermediate": pixels == 205}
    fig, ax = plt.subplots(figsize=(3.2, 2.3))
    fig.subplots_adjust(left=0.18, right=0.96, bottom=0.24, top=0.92)
    # PGM row zero is the top; YAML origin is the lower-left map corner.
    ax.imshow(pixels, cmap="gray", vmin=0, vmax=255, interpolation="nearest",
              extent=(xmin, xmax, ymin, ymax), origin="upper")
    ax.scatter([0], [0], marker="+", s=26, color=BLUE, linewidths=0.8)
    ax.set(xlabel="x (m)", ylabel="y (m)", xlim=(xmin, xmax), ylim=(ymin, ymax))
    ax.set_aspect("equal")
    ax.set_xticks([-5, 0, 5])
    ax.set_yticks([-5, 0, 5])
    ax.tick_params(labelsize=8.5, pad=2)
    ax.xaxis.labelpad, ax.yaxis.labelpad = 2, 2
    handles = [Patch(facecolor="black", label="Occupied"),
               Patch(facecolor=str(254 / 255), edgecolor="#B7BAC0", linewidth=0.5, label="Free"),
               Patch(facecolor=str(205 / 255), label="Gray (205)"),
               Line2D([], [], color=BLUE, marker="+", linestyle="none", markersize=5, label="(0, 0)")]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False,
               handlelength=0.8, handletextpad=0.3, columnspacing=0.7,
               borderaxespad=0.1, bbox_to_anchor=(0.51, 0.005), fontsize=8.5)
    save_figure(fig, "map_results")
    return {
        "sources": ["map.pgm", "map.yaml"], "render": "Saved PGM, not an RViz screenshot; no smoothing or crop.",
        "resolution_m_per_pixel": resolution, "origin_xy_yaw": origin,
        "width_pixels": width, "height_pixels": height,
        "extent_m": {"x": [xmin, xmax], "y": [ymin, ymax]},
        "pixel_values_counts": {str(int(v)): int(c) for v, c in zip(*np.unique(pixels, return_counts=True))},
        "occupancy_threshold": float(metadata["occupied_thresh"]),
        "free_threshold": float(metadata["free_thresh"]),
        "saved_image_class_counts": {key: int(mask.sum()) for key, mask in image_masks.items()},
        "saved_image_class_fractions": {key: float(mask.mean()) for key, mask in image_masks.items()},
        "yaml_reload_class_counts": {"occupied": int(occupied.sum()), "free": int(free.sum()), "unknown": int(unknown.sum())},
        "yaml_reload_class_fractions": {"occupied": float(occupied.mean()), "free": float(free.mean()), "unknown": float(unknown.mean())},
        "metadata_limitation": "Gray205 (saved unknown/intermediate) has occupancy50/255=0.196078, below YAMLfree_thresh0.25, so it reloads asfree. Neither source file was modified.",
        "classification_reference": "https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp",
    }


if __name__ == "__main__":
    FIGURES.mkdir(parents=True, exist_ok=True)
    results = {"lidar": lidar_results(), "map": map_results()}
    (ROOT / "analysis" / "lidar_mapping_metrics.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))
