# Lab 4 report figures (fig_map, fig_runs, fig_heuristics) and the numbers quoted in the report (numbers.json).
# Usage, from lab4/:  python3 report/make_figures.py runs report/figures --map room      (needs numpy, pandas, matplotlib, scipy, scikit-learn)
import os, glob, json, argparse
import numpy as np, pandas as pd
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument('runs'); ap.add_argument('out'); ap.add_argument('--map', default='room')
ap.add_argument('--laser_sig', type=float, default=0.4, help='same value as planner.py')
ap.add_argument('--keep', nargs='+', default=['165419', '170619'], help='runs to show (the ones that reached their goal)')
ap.add_argument('--xlim', type=float, nargs=2, default=[-1.5, 7.5]); ap.add_argument('--ylim', type=float, nargs=2, default=[-2.5, 3.5])
A = ap.parse_args(); os.makedirs(A.out, exist_ok=True)

INK, MUTED, ODOM, EKF, PATH, GRID = '#0b0b0b', '#52514e', '#3d3c39', '#2a78d6', '#eb6834', '#e3e2dd'
plt.rcParams.update({'font.size': 7, 'axes.titlesize': 7, 'axes.labelsize': 7, 'xtick.labelsize': 6.5, 'ytick.labelsize': 6.5,
    'legend.fontsize': 6.3, 'axes.linewidth': 0.6, 'axes.edgecolor': MUTED, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.5, 'axes.axisbelow': True, 'axes.spines.top': False, 'axes.spines.right': False,
    'legend.framealpha': 0.9, 'legend.edgecolor': 'none', 'pdf.fonttype': 42, 'savefig.dpi': 300, 'axes.titlepad': 3})

def load(f):
    d = pd.read_csv(f, skipinitialspace=True)
    d = d.loc[:, [c for c in d.columns if c.strip() and not c.startswith('Unnamed')]]
    d.columns = [c.strip() for c in d.columns]
    return d.apply(pd.to_numeric, errors='coerce')

def read_map(name):
    meta = dict(l.split(':', 1) for l in open(name + '.yaml') if ':' in l)
    res = float(meta['resolution']); ox, oy = [float(v) for v in meta['origin'].strip(' []\n').split(',')[:2]]
    with open(name + '.pgm', 'rb') as fh:
        f = []
        while len(f) < 4:
            l = fh.readline(); f += [] if l.startswith(b'#') else l.split()
        w, h = int(f[1]), int(f[2]); img = np.frombuffer(fh.read(w * h), np.uint8).reshape(h, w)
    return img, res, [ox, ox + w * res, oy, oy + h * res]

img, res, ext = read_map(A.map)
NUM = {'laser_sig': A.laser_sig, 'blocked_radius_m': round(0.668 * A.laser_sig, 3)}

# ---------------------------------------------------------------- Fig: map and cost map (same Gaussian as mapUtilities.make_likelihood_field)
try:
    from scipy.ndimage import distance_transform_edt
    cost = np.exp(-(distance_transform_edt(img >= 10) * res) ** 2 / (2 * A.laser_sig ** 2))
except ImportError:
    cost = None
fig, ax = plt.subplots(1, 2 if cost is not None else 1, figsize=(7.1, 2.8), squeeze=False); ax = ax[0]
ax[0].imshow(img, cmap='gray', extent=ext); ax[0].set(title='(a) SLAM map (room.pgm)', xlabel='x [m]', ylabel='y [m]')
if cost is not None:
    im = ax[1].imshow(cost, cmap='magma_r', extent=ext, vmin=0, vmax=1)
    ax[1].contour(np.flipud(cost) > 0.8, levels=[0.5], colors=[EKF], linewidths=0.6, extent=ext)
    ax[1].set(title=rf'(b) Cost map, $\sigma$={A.laser_sig} m (blue: cost > 0.8, walls for A*)', xlabel='x [m]')
    fig.colorbar(im, ax=ax[1], fraction=0.04, pad=0.02)
fig.tight_layout(); fig.savefig(f'{A.out}/fig_map.pdf'); plt.close(fig)

# ---------------------------------------------------------------- Fig: the runs that reached their goal
# Each log is cut at the closest approach to the goal (goal reached); 165419's pose was reset afterwards.
runs = [glob.glob(f'{A.runs}/*{r}*_pose.csv')[0] for r in A.keep]
tasks = []   # (label, start, goal) of each run, replanned with both heuristics below
fig, ax = plt.subplots(1, len(runs), figsize=(7.1, 2.5), squeeze=False); ax = ax[0]
for k, (a, f) in enumerate(zip(ax, runs)):
    name = os.path.basename(f)[:-len('_pose.csv')]; stem = f[:-len('_pose.csv')]; lbl = name.split('_')[0][-6:]
    p, path = load(f), load(stem + '_path.csv')
    gx, gy = path.x.iloc[-1], path.y.iloc[-1]
    end = int(np.hypot(p.odom_x - gx, p.odom_y - gy).argmin()); p = p.iloc[:end + 1]
    t = (p.stamp - p.stamp.iloc[0]) / 1e9
    a.imshow(img, cmap='gray', extent=ext)
    a.plot(path.x, path.y, '--', color=PATH, lw=1.0, label='A* path (Euclidean)')
    a.plot(p.odom_x, p.odom_y, color=ODOM, lw=0.9, label='odometry')
    a.plot(p.kf_x, p.kf_y, color=EKF, lw=0.9, label='EKF')
    a.plot(p.odom_x.iloc[0], p.odom_y.iloc[0], 'o', color='tab:green', ms=3, label='start')
    a.plot(gx, gy, 'x', color=PATH, ms=5, mew=1.5, label='goal')
    a.set_title(f'({"ab"[k]}) Run {lbl}', loc='left'); a.set_aspect('equal'); a.set(xlim=A.xlim, ylim=A.ylim, xlabel='x [m]', ylabel='y [m]')
    seg = np.hypot(np.diff(path.x), np.diff(path.y)).sum()
    NUM[name] = dict(time_to_goal_s=round(float(t.iloc[-1]), 1), path_len_m=round(float(seg), 2), path_cells=len(path),
                     start=[round(float(path.x.iloc[0]), 2), round(float(path.y.iloc[0]), 2)], goal=[round(float(gx), 2), round(float(gy), 2)],
                     final_odom_to_goal_m=round(float(np.hypot(p.odom_x.iloc[-1] - gx, p.odom_y.iloc[-1] - gy)), 3),
                     final_ekf_to_goal_m=round(float(np.hypot(p.kf_x.iloc[-1] - gx, p.kf_y.iloc[-1] - gy)), 3),
                     max_ekf_odom_gap_m=round(float(np.hypot(p.kf_x - p.odom_x, p.kf_y - p.odom_y).max()), 3))
    tasks.append((lbl, (path.x.iloc[0], path.y.iloc[0]), (gx, gy), path, float(t.iloc[-1])))
ax[0].legend(loc='lower left', fontsize=5.5)
fig.tight_layout(); fig.savefig(f'{A.out}/fig_runs.pdf'); plt.close(fig)

# ---------------------------------------------------------------- Fig: Euclidean vs Manhattan, replanned offline with planner.py
# planner.py -> mapUtilities.mapManipulator -> a_star.search, unchanged. Only the ROS imports are stubbed (no rclpy on a laptop);
# the planner never publishes, so the stubs are never used.
import sys, types
class _Stub(types.ModuleType):
    def __getattr__(self, n):
        if n.startswith('__'): raise AttributeError(n)
        return type(n, (), {'__init__': lambda self, *a, **k: None})
for m in ['rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time', 'sensor_msgs', 'sensor_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
          'geometry_msgs', 'geometry_msgs.msg', 'std_msgs', 'std_msgs.msg', 'tf_transformations']:
    sys.modules.setdefault(m, _Stub(m))
LAB4 = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, LAB4)
cwd = os.getcwd(); os.chdir(LAB4)
from planner import planner, TRAJECTORY_PLANNER
HC = {'euclidean': PATH, 'manhattan': '#1baf7a'}
fig, ax = plt.subplots(1, len(tasks), figsize=(7.1, 2.5), squeeze=False); ax = ax[0]
for k, (a, (lbl, start, goal, logged, t_done)) in enumerate(zip(ax, tasks)):
    a.imshow(img, cmap='gray', extent=ext)
    NUM[f'heuristics_{lbl}'] = {}
    for h in ['euclidean', 'manhattan']:
        pl = planner(TRAJECTORY_PLANNER, A.map, heuristic=h); pl.initTrajectoryPlanner()
        xy = np.array(pl.trajectory_planner(start, goal))
        L = float(np.hypot(*np.diff(xy, axis=0).T).sum())
        # time to completion only exists for the heuristic the robot drove (Euclidean); Manhattan was planned, not driven
        done = round(t_done, 1) if h == 'euclidean' else None
        a.plot(xy[:, 0], xy[:, 1], color=HC[h], lw=1.2, ls='-' if h == 'euclidean' else '--',
               label=f'{h.capitalize()}: {L:.2f} m, ' + (f'driven in {done:.0f} s' if done else 'not driven'))
        NUM[f'heuristics_{lbl}'][h] = dict(path_len_m=round(L, 2), cells=len(xy), time_to_completion_s=done)
        if h == 'euclidean':   # sanity check: same planner + map + sigma should reproduce the path logged in lab
            NUM[f'heuristics_{lbl}']['matches_logged_path'] = bool(len(xy) == len(logged) and np.allclose(xy, logged[['x', 'y']].values, atol=1e-6))
    a.plot(*start, 'o', color='tab:green', ms=3); a.plot(*goal, 'x', color='k', ms=5, mew=1.5)
    a.set_title(f'({"ab"[k]}) Start/goal of run {lbl}', loc='left'); a.set_aspect('equal'); a.set(xlim=A.xlim, ylim=A.ylim, xlabel='x [m]', ylabel='y [m]')
    a.legend(loc='lower left', fontsize=5.5)
os.chdir(cwd)
fig.tight_layout(); fig.savefig(f'{A.out}/fig_heuristics.pdf'); plt.close(fig)

json.dump(NUM, open(f'{A.out}/numbers.json', 'w'), indent=1)
print(json.dumps(NUM, indent=1))
