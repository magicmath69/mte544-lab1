# Full Lab 1 report: source and verification notes

The editable report is `report/report.tex`, with one owned section per file in
`report/sections/`. Compile from `report/` with
`pdflatex -interaction=nonstopmode -halt-on-error report.tex`, twice after changing
cross-references. The cover is separate from the two numbered content pages.

Recorded inputs come from `magicmath69/mte544-lab1`, report branch commit
`aef9d8d0c6cf59e9748cb3937bd44569734c0aef`. The source CSVs, saved PGM/YAML, and
robot-control code have not been edited. Recreate the figures from the checkout
root with:

```
python analysis/odometry_results.py
python analysis/imu_results.py
python analysis/lidar_mapping_results.py
```

These helpers require NumPy, Matplotlib, and Pillow. Results are retained in the
three adjacent JSON files. Figures are vector PDFs plus PNGs for visual review.
All time stamps are parsed as integer nanoseconds and differenced before
conversion. No temporal smoothing or resampling is used. LiDAR excludes invalid
returns before projection while preserving original beam indices; no unlogged
sensor range limits are invented.

The report uses raw logged coordinate frames. Independent start times are not
cross-sensor synchronization. Yaw unwrapping removes representation changes,
not physical motion. Circle fitting describes odometry geometry rather than
localization accuracy. Acceleration variation includes actual motion and any
measurement effects and is not a separately estimated sensor-noise parameter.

For the spiral description, the outer-loop radius is fitted to the recorded
XY poses at elapsed times >=60 s (617 samples, 60.003--90.804 s). The fitted
radius is 0.470423 m, radial RMS residual 0.003762 m. An independent fit starting
at 55 s gives 0.469627 m, supporting the reported approximate radius of 0.470 m.
This is the late-loop radius, not a single radius for the entire spiral.

The map plot preserves PGM pixel values and YAML metric extent. Gray 205 is
labelled separately from confirmed free and occupied pixels. The supplied YAML
`free_thresh: 0.25` is greater than `(255-205)/255`; the official Nav2 Humble
trinary loader would therefore classify these pixels as free. This remains a
source metadata limitation; the supplied map files have not been changed.
Primary implementation: https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp

The report's brief map description names the ROS 2 SLAM, RViz, and Nav2 map-saver
workflow documented in the lab instructions. The robot's bottom-right location
was supplied by Joe; the saved PGM does not independently record the robot pose.

Course reference checked for interpretation: Sensing and Measurements I,
odometry, gyroscope, and accelerometer sections. The report follows the lab
manual and rubric rather than including the former ME202 template material.

The cover uses the IDs explicitly supplied in the source/conversation:
Hunter 20997503, Joe 20998545, Jacob 21004619. Robot number is 7; the station
number is intentionally blank at the user's request. Platform-specific acquisition
details were not established, so the report does not invent them.

This report, the figures, and the analysis helpers were created with Codex
assistance. They are not represented as independently student-authored work.
Retain this conversation as the prompt/response record. The Assignment 1 AI
restrictions were discussed earlier; no lab-specific permission was established
by the available repository documents. This note records provenance, not approval.

Verification covers finite sensor values where applicable, monotonic timestamps,
independent odometry and IMU calculations, original scan indexing, map dimensions,
numeric/prose agreement, PDF cross-references, exact page count, and visual review
of every rendered page. Commit and publication status are recorded in Git history.
