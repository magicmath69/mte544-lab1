# Lab 3 report figures (fig_gains, fig_spiral, fig_ratio, fig_point) and the numbers quoted in the report.
# Usage, from lab3/:  python3 report/make_figures.py runs_lab3 report/figures      (needs numpy, pandas, matplotlib)
# Every figure is built from the logged CSVs; "re-filtered" curves replay the same EKF equations offline on a log's measurements.
import sys, os, re, glob, json
import numpy as np, pandas as pd
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

D, OUT = sys.argv[1], sys.argv[2]
INK, MUTED, MEAS, RAW, EKF, PS, GRID = '#0b0b0b', '#52514e', '#3d3c39', '#b4b3ad', '#2a78d6', '#eb6834', '#e3e2dd'
plt.rcParams.update({'font.size': 7, 'axes.titlesize': 7, 'axes.labelsize': 7, 'xtick.labelsize': 6.5, 'ytick.labelsize': 6.5,
    'legend.fontsize': 6.3, 'axes.linewidth': 0.6, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'text.color': INK,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.major.width': 0.5, 'ytick.major.width': 0.5, 'xtick.major.size': 2.5,
    'ytick.major.size': 2.5, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.5, 'axes.axisbelow': True,
    'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': True, 'legend.framealpha': 0.9,
    'legend.edgecolor': 'none', 'legend.borderpad': 0.25, 'legend.handlelength': 1.8, 'legend.labelspacing': 0.25,
    'pdf.fonttype': 42, 'savefig.dpi': 300, 'axes.titlepad': 3, 'axes.labelpad': 2})
R_PS = np.array([2e-5, 2e-4, 0.05, 0.06]); TWIN = 25.0

def load(f):
    d = pd.read_csv(f, skipinitialspace=True)
    d = d.loc[:, [c for c in d.columns if c.strip() and not c.startswith('Unnamed')]]
    d.columns = [c.strip() for c in d.columns]
    d = d.apply(pd.to_numeric, errors='coerce')
    d['t'] = (d.stamp - d.stamp.iloc[0]) / 1e9
    d['ox'] = d.odom_x - d.odom_x.iloc[0]; d['oy'] = d.odom_y - d.odom_y.iloc[0]
    th = np.unwrap(d.odom_th.values); d['oth'] = th - th[0]
    return d

def replay(d, Q, R, dt=0.1, stamps=False):
    """Same equations as kalman_filter.py, run on the logged z = [odom_v, odom_w, imu_ax, imu_ay]."""
    Q = np.diag(Q) if np.ndim(Q) == 1 else Q * np.eye(6); R = np.diag(R) if np.ndim(R) == 1 else R * np.eye(4)
    x = np.zeros(6); P = Q.copy(); X = []; Ks = []; Ps = []
    tt = d.t.values
    for k, z in enumerate(d[['odom_v', 'odom_w', 'imu_ax', 'imu_ay']].values):
        if stamps: dt = tt[k] - tt[k-1] if k else 0.0
        _, _, th, w, v, vd = x
        A = np.array([[1, 0, -v*np.sin(th)*dt, 0, np.cos(th)*dt, 0], [0, 1, v*np.cos(th)*dt, 0, np.sin(th)*dt, 0],
                      [0, 0, 1, dt, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, dt], [0, 0, 0, 0, 0, 1.]])
        x = np.array([x[0] + v*np.cos(th)*dt, x[1] + v*np.sin(th)*dt, th + w*dt, w, v + vd*dt, vd])
        _, _, th, w, v, vd = x
        C = np.array([[0, 0, 0, 0, 1, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1], [0, 0, 0, v, w, 0.]])
        P = A @ P @ A.T + Q; K = P @ C.T @ np.linalg.inv(C @ P @ C.T + R)
        x = x + K @ (z - np.array([v, w, vd, v*w])); P = (np.eye(6) - K @ C) @ P
        X.append(x); Ks.append(K); Ps.append(np.diag(P))
    return pd.DataFrame(np.array(X), columns=['x', 'y', 'th', 'w', 'v', 'vd']), np.array(Ks), np.array(Ps)

def met(d, e=None, tmax=None):
    """EKF-vs-measurement numbers; e=None uses the logged estimate."""
    if e is None: e = pd.DataFrame({'x': d.kf_x, 'y': d.kf_y, 'th': d.kf_th, 'w': d.kf_w, 'v': d.kf_vx, 'vd': d.kf_ax})
    m = (d.t <= tmax).values if tmax else np.ones(len(d), bool)
    gap = np.hypot(e.x.values - d.ox.values, e.y.values - d.oy.values)[m]
    return dict(rms_v=float(np.sqrt(np.mean((e.v.values - d.odom_v.values)[m]**2))),
                rms_w=float(np.sqrt(np.mean((e.w.values - d.odom_w.values)[m]**2))),
                gap_end=float(gap[-1]), gap_max=float(gap.max()),
                sd_vd=float(np.std(e.vd.values[m])), sd_ax=float(np.std(d.imu_ax.values[m])),
                sd_ay_est=float(np.std((e.v*e.w).values[m])), sd_ay=float(np.std(d.imu_ay.values[m])))

runs = {}
for f in sorted(glob.glob(D + '/*.csv')):
    b = os.path.basename(f); m = re.match(r'\d+-(\d+)_q([\d.]+)_r(.+?)(-+good|-stealtest)?\.csv', b)
    runs[m.group(1)] = dict(f=f, q=float(m.group(2)), r=m.group(3), tag=(m.group(4) or '').strip('-'), d=load(f))
SP = [k for k, v in runs.items() if v['tag'] == 'good']                       # 7 spiral runs
PT = [k for k, v in runs.items() if v['tag'] == '' and v['d'].t.iloc[-1] > 10 and 'e-5' not in v['r']]   # 7 uniform point runs
PSRUN = [k for k, v in runs.items() if 'e-5' in v['r']][0]
NUM = {}   # numbers quoted in the report
def lab(ax, s): ax.text(0.02, 0.97, s, transform=ax.transAxes, va='top', ha='left', fontsize=6.2, color=INK,
                        bbox=dict(fc='white', ec='none', alpha=0.85, pad=1))
def tag(ax, s): ax.set_title(s, loc='left', fontweight='bold')

# ---------------------------------------------------------------- Fig A: covariance and gains (baseline spiral)
base = runs['160605']['d']
e, Ks, Ps = replay(base, 0.5, 0.5)
fig, ax = plt.subplots(1, 3, figsize=(7.1, 1.5), gridspec_kw=dict(wspace=0.36, left=0.065, right=0.99, top=0.85, bottom=0.24))
sty = [('x', 0, MEAS, '-'), ('y', 1, MEAS, '--'), (r'$\theta$', 2, MEAS, ':'), (r'$\omega$', 3, EKF, '-'), ('v', 4, EKF, '--'), (r'$\dot v$', 5, EKF, ':')]
for n, i, c, ls in sty: ax[0].semilogy(base.t, np.sqrt(Ps[:, i]), color=c, ls=ls, lw=1.1, label=n)
ax[0].set(xlabel='Time [s]', ylabel=r'$\sqrt{P_{ii}}$  [state units]', xlim=(0, 30))
tag(ax[0], r'(a) Uncertainty of each state, $Q=R=0.5\,I$')
ax[0].legend(ncol=2, loc='center right', bbox_to_anchor=(1.0, 0.36), title='not measured | measured', title_fontsize=6, columnspacing=1)
rat = np.logspace(-2, 2, 21); G = []
for q in rat:
    _, K, _ = replay(base, q, 1.0); G.append(K[200])
G = np.array(G); i1 = np.argmin(abs(rat - 1))
gl = [(r'$v \leftarrow$ odom $v$', 4, 0, EKF, '-'), (r'$\omega \leftarrow$ odom $\omega$', 3, 1, EKF, '--'),
      (r'$\dot v \leftarrow$ IMU $a_x$', 5, 2, PS, '-'), (r'$v \leftarrow$ IMU $a_y$', 4, 3, PS, '--')]
for n, i, j, c, ls in gl: ax[1].semilogx(rat, G[:, i, j], color=c, ls=ls, lw=1.2, label=n)
for q in (0.25, 0.5, 1, 2, 4): ax[1].axvline(q, color=RAW, lw=0.5, zorder=0)
ax[1].set(xlabel=r'$Q/R$ (scalar ratio)', ylabel='Kalman gain entry', ylim=(0, 1.02))
tag(ax[1], r'(b) Gain on the circle vs $Q/R$'); ax[1].legend(loc='upper left')
ax[1].text(1, 0.03, 'tested in lab', ha='center', fontsize=5.8, color=MUTED, bbox=dict(fc='white', ec='none', pad=0.5))
_, Kp, _ = replay(base, np.full(6, 0.25), R_PS)
names = [r'$v\!\leftarrow\!v$', r'$\omega\!\leftarrow\!\omega$', r'$\dot v\!\leftarrow\!a_x$', r'$v\!\leftarrow\!a_y$']
gu = [G[i1][i, j] for _, i, j, _, _ in gl]; gp = [Kp[200][i, j] for _, i, j, _, _ in gl]
xx = np.arange(4)
ax[2].bar(xx - 0.19, gu, 0.34, color=EKF, label=r'uniform, $Q/R=1$'); ax[2].bar(xx + 0.19, gp, 0.34, color=PS, label=r'per-sensor $R$')
for x0, g in zip(xx - 0.19, gu): ax[2].text(x0, g + 0.02, f'{g:.2f}', ha='center', fontsize=5.8)
for x0, g in zip(xx + 0.19, gp): ax[2].text(x0, max(g, 0) + 0.02, f'{g:.2f}' if abs(g) >= 0.005 else f'{g:.3f}', ha='center', fontsize=5.8)
ax[2].set_xticks(xx); ax[2].set_xticklabels(names); ax[2].set(ylabel='Kalman gain entry', ylim=(0, 1.45), xlabel='state $\\leftarrow$ measurement')
ax[2].grid(axis='x', visible=False); tag(ax[2], '(c) Uniform vs per-sensor $R$'); ax[2].legend(loc='upper center', ncol=2, columnspacing=1)
fig.savefig(OUT + '/fig_gains.pdf'); plt.close(fig)
NUM['gains_uniform_ratio1'] = dict(zip(['v_v', 'w_w', 'vd_ax', 'v_ay'], map(float, gu))); NUM['gains_persensor'] = dict(zip(['v_v', 'w_w', 'vd_ax', 'v_ay'], map(float, gp)))
NUM['sigma_end_base'] = dict(zip(['x', 'y', 'th', 'w', 'v', 'vd'], map(float, np.sqrt(Ps[-1])))); NUM['sigma_xy_at25'] = float(np.sqrt(Ps[np.argmin(abs(base.t-25)), 0]))

# ---------------------------------------------------------------- Fig 1: spiral, measured vs estimated, 5 settings
HL = {}
cols = [('161029', 'Q halved'), ('161116', 'R doubled'), ('160605', 'baseline'), ('160758', 'Q doubled'), ('161224', 'R halved')]
fig = plt.figure(figsize=(7.1, 4.8))
go = fig.add_gridspec(2, 1, height_ratios=[1.3, 3.7], hspace=0.31, left=0.07, right=0.995, top=0.855, bottom=0.07)
g0 = go[0].subgridspec(1, 5, wspace=0.12); g1 = go[1].subgridspec(4, 5, wspace=0.12, hspace=0.38)
fig.suptitle('Spiral on the robot: measured (odometry, IMU) vs EKF estimate, first 25 s of each run', fontsize=8, fontweight='bold', y=0.997)
rowspec = [('Forward speed $v$', 'v [m/s]', (-0.45, 0.95)), (r'Yaw rate $\omega$', r'$\omega$ [rad/s]', (-0.1, 1.6)),
           (r'Acceleration $a_x$ ($=\dot v$)', r'$a_x$ [m/s$^2$]', (-1.6, 1.9)), (r'Acceleration $a_y$ ($=v\omega$)', r'$a_y$ [m/s$^2$]', (-1.6, 1.9))]
for c, (k, what) in enumerate(cols):
    r = runs[k]; d = r['d'][r['d'].t <= TWIN]; M = met(r['d'], tmax=TWIN); NUM['spiral_' + k] = dict(q=r['q'], r=float(r['r']), **M)
    a = fig.add_subplot(g0[0, c]); a.plot(d.kf_x, d.kf_y, color=EKF, lw=1.2, label='EKF estimate'); a.plot(d.ox, d.oy, color=MEAS, lw=0.9, ls='--', label='Odometry (measured)')
    a.plot(d.ox.iloc[-1], d.oy.iloc[-1], 'o', ms=3.2, color=MEAS, mec='white', mew=0.5); a.plot(d.kf_x.iloc[-1], d.kf_y.iloc[-1], 'o', ms=3.2, color=EKF, mec='white', mew=0.5)
    a.set_aspect('equal', adjustable='box'); a.set(xlim=(-1.35, 1.35), ylim=(-1.15, 2.3), xlabel='x [m]'); a.set_xticks([-1, 0, 1]); a.set_yticks([0, 1, 2])
    a.set_title(f"$Q={r['q']:g}\\,I$, $R={float(r['r']):g}\\,I$\n{what}, $Q/R={r['q']/float(r['r']):g}$", fontsize=7)
    a.text(0.5, 0.03, f"gap at 25 s: {M['gap_end']:.2f} m", transform=a.transAxes, ha='center', fontsize=6.2, bbox=dict(fc='white', ec='none', alpha=0.85, pad=1))
    if c == 0: a.set_ylabel('Path:  y [m]')
    else: a.set_yticklabels([])
    series = [(d.odom_v, d.kf_vx, f"RMS diff. {M['rms_v']:.2f} m/s", 'Odometry', 'EKF'), (d.odom_w, d.kf_w, f"RMS diff. {M['rms_w']:.2f} rad/s", 'Odometry', 'EKF'),
              (d.imu_ax, d.kf_ax, f"std: IMU {M['sd_ax']:.2f}, EKF {M['sd_vd']:.2f}", 'IMU', r'EKF $\dot v$'),
              (d.imu_ay, d.kf_ay, f"std: IMU {M['sd_ay']:.2f}, EKF {M['sd_ay_est']:.2f}", 'IMU', r'EKF $v\omega$')]
    for i, ((ttl, yl, ylim), (ms, es, note, lm, le)) in enumerate(zip(rowspec, series)):
        a = fig.add_subplot(g1[i, c]); raw = i >= 2
        if raw: a.plot(d.t, ms, color=RAW, lw=0.6, label='IMU (measured)')
        a.plot(d.t, es, color=EKF, lw=0.9, label='EKF estimate')
        if not raw: a.plot(d.t, ms, color=MEAS, lw=0.9, ls='--', label='Odometry (measured)')
        if i == 3: a.plot(d.t, d.odom_v * d.odom_w, color=MEAS, lw=0.9, ls=':', label=r'Odometry $v\omega$ (expected $a_y$)')
        if c == 0 and i in (0, 3): HL.update(dict(zip(a.get_legend_handles_labels()[1], a.get_legend_handles_labels()[0])))
        a.set(xlim=(0, TWIN), ylim=ylim); a.set_title(ttl, fontsize=6.3, pad=1.5); a.set_xticks([0, 10, 20])
        a.text(0.98, 0.07, note, transform=a.transAxes, ha='right', fontsize=6, bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.6))
        if c == 0: a.set_ylabel(yl)
        else: a.set_yticklabels([])
        if i == 3: a.set_xlabel('Time [s]')
        else: a.set_xticklabels([])
order = ['Odometry (measured)', 'IMU (measured)', r'Odometry $v\omega$ (expected $a_y$)', 'EKF estimate']
fig.legend([HL[k] for k in order], order, loc='upper center', bbox_to_anchor=(0.53, 0.978), ncol=4, columnspacing=1.8, fontsize=6.8)
fig.savefig(OUT + '/fig_spiral.pdf'); plt.close(fig)

# ---------------------------------------------------------------- Fig 2: effect of Q/R, all 7 spiral runs
rat = np.logspace(-2, 2, 25)
keys = [('rms_v', r'RMS($\hat v - v_{odom}$) [m/s]', '(a) Speed error'), ('rms_w', r'RMS($\hat\omega - \omega_{odom}$) [rad/s]', '(b) Yaw-rate error'),
        ('gap_max', 'max EKF-odometry gap [m]', '(c) Position gap'), ('nz', r'std($\hat{\dot v}$) / std(IMU $a_x$)', '(d) IMU noise passed to $\\dot v$')]
curves = {k: [] for k in SP}; psv = {k: None for k in SP}
for k in SP:
    d = runs[k]['d']
    for q in rat:
        m = met(d, replay(d, q, 1.0)[0], TWIN); m['nz'] = m['sd_vd'] / m['sd_ax']; curves[k].append(m)
    m = met(d, replay(d, np.full(6, 0.25), R_PS)[0], TWIN); m['nz'] = m['sd_vd'] / m['sd_ax']; psv[k] = m
NUM['spiral_persensor_refiltered'] = {k: psv[k] for k in SP}
psdt = float(np.median([met(runs[k]['d'], replay(runs[k]['d'], np.full(6, 0.25), R_PS, stamps=True)[0], TWIN)['gap_max'] for k in SP])); NUM['ps_stampdt_median_gap_max'] = psdt
NUM['logged_median_gap_max'] = float(np.median([met(runs[k]['d'], tmax=TWIN)['gap_max'] for k in SP]))
NUM['dt_stats'] = {k: dict(first8_mean=float(np.diff(runs[k]['d'].t.values)[:8].mean()), min=float(np.diff(runs[k]['d'].t.values).min()), max=float(np.diff(runs[k]['d'].t.values).max()), mean=float(np.diff(runs[k]['d'].t.values).mean())) for k in SP + PT}
fig, ax = plt.subplots(1, 4, figsize=(7.1, 2.0), gridspec_kw=dict(wspace=0.42, left=0.065, right=0.995, top=0.73, bottom=0.195))
fig.suptitle('Spiral: what the $Q/R$ ratio does (first 25 s of each of the 7 runs)', fontsize=8, fontweight='bold', y=0.995)
for a, (key, yl, ttl) in zip(ax, keys):
    Y = np.array([[m[key] for m in curves[k]] for k in SP])
    for j, k in enumerate(SP): a.semilogx(rat, Y[j], color=RAW, lw=0.6, label='each log re-filtered' if j == 0 else None)
    a.semilogx(rat, np.median(Y, 0), color=MEAS, lw=1.2, label='median of the 7 logs')
    for j, k in enumerate(SP):
        r = runs[k]; m = met(r['d'], tmax=TWIN); m['nz'] = m['sd_vd'] / m['sd_ax']
        a.plot(r['q'] / float(r['r']), m[key], 'o', ms=4.2, color=EKF, mec='white', mew=0.6, zorder=5, label='as run on the robot' if j == 0 else None)
    p = np.median([psv[k][key] for k in SP]); NUM['ps_median_' + key] = float(p)
    a.axhline(p, color=PS, lw=1.1, ls='--', label='per-sensor $R$ (median, re-filtered)')
    a.set(xlabel='$Q/R$', ylabel=yl, xlim=(0.01, 100), ylim=(0, 0.8 if key == 'gap_max' else None)); tag(a, ttl); a.set_xticks([0.01, 0.1, 1, 10, 100])
    if key == 'gap_max': a.axhline(psdt, color=PS, lw=1.0, ls=':'); a.text(0.012, 0.125, 'dotted: per-sensor $R$,\n$dt$ from stamps', ha='left', va='bottom', fontsize=5.4, linespacing=0.95)
    if key in ('rms_v', 'rms_w'): a.text(90, p + 0.004 * (1 if key == 'rms_v' else 0.7), 'per-sensor $R$: < 0.001', ha='right', va='bottom', fontsize=5.6)
    NUM['median_' + key] = dict(zip([f'{x:.3g}' for x in rat], map(float, np.median(Y, 0))))
h, l = ax[0].get_legend_handles_labels()
fig.legend(h, l, loc='upper center', bbox_to_anchor=(0.53, 0.935), ncol=4, columnspacing=1.5, fontsize=6.3)
fig.savefig(OUT + '/fig_ratio.pdf'); plt.close(fig)
# equivalence check quoted in the text: Q=1,R=0.5 vs Q=0.5,R=0.25 on the same log
ea, eb = replay(base, 1, 0.5)[0], replay(base, 0.5, 0.25)[0]; NUM['equiv_maxdiff'] = float(np.abs(ea.values - eb.values).max())
NUM['replay_maxdiff_all_logs'] = float(max(np.abs(replay(v['d'], np.full(6, v['q']), R_PS if 'e-5' in v['r'] else np.full(4, float(v['r'])))[0][['x', 'y', 'v', 'w']].values
                                                    - v['d'][['kf_x', 'kf_y', 'kf_vx', 'kf_w']].values).max() for v in runs.values()))

# ---------------------------------------------------------------- Fig 3: point controller
FIN = '164131'; TCUT = 13.6
fig = plt.figure(figsize=(7.1, 2.1))
gs = fig.add_gridspec(2, 4, width_ratios=[1, 1.05, 1, 1.25], wspace=0.42, hspace=0.25, left=0.06, right=0.995, top=0.81, bottom=0.21)
fig.suptitle('Point controller to (-1, -1) m with the EKF pose in the loop', fontsize=8, fontweight='bold', y=0.995)
def path(a, d, ttl, ecol, note):
    a.plot(d.ox, d.oy, color=MEAS, lw=0.9, label='Odometry (measured)'); a.plot(d.kf_x, d.kf_y, color=ecol, lw=1.1, ls='--', label='EKF estimate')
    a.plot(-1, -1, marker='*', ms=7, color=INK, mec='white', mew=0.4, ls='none', label='Goal'); a.plot(0, 0, marker='s', ms=3, color=MUTED, ls='none', label='Start')
    a.plot(d.ox.iloc[-1], d.oy.iloc[-1], 'o', ms=3.2, color=MEAS, mec='white', mew=0.5); a.plot(d.kf_x.iloc[-1], d.kf_y.iloc[-1], 'o', ms=3.2, color=ecol, mec='white', mew=0.5)
    a.set_aspect('equal', adjustable='datalim'); a.set(xlabel='x [m]', ylabel='y [m]', xlim=(-1.25, 0.45), ylim=(-1.25, 0.25)); tag(a, ttl)
    a.text(0.97, 0.05, note, transform=a.transAxes, ha='right', fontsize=6, bbox=dict(fc='white', ec='none', alpha=0.85, pad=1))
d = runs[FIN]['d']; M = met(d); NUM['point_final'] = dict(T=float(d.t.iloc[-1]), odom_end=[float(d.ox.iloc[-1]), float(d.oy.iloc[-1])], kf_end=[float(d.kf_x.iloc[-1]), float(d.kf_y.iloc[-1])], **M)
a0 = fig.add_subplot(gs[:, 0]); path(a0, d, '(a) Path, $Q=0.25\\,I$, $R=1\\,I$', EKF, f"gap at goal: {M['gap_end']*100:.0f} cm")
a0.legend(loc='upper left', bbox_to_anchor=(-0.02, 1.0), fontsize=5.6, handlelength=1.5, borderaxespad=0.1)
for i, (ms, es, yl, ttl) in enumerate([(d.odom_v, d.kf_vx, 'v [m/s]', '(b) $v$ and $\\omega$, same run'), (d.odom_w, d.kf_w, r'$\omega$ [rad/s]', None)]):
    a = fig.add_subplot(gs[i, 1]); a.plot(d.t, ms, color=MEAS, lw=0.9, label='Odometry (measured)'); a.plot(d.t, es, color=EKF, lw=1.0, ls='--', label='EKF estimate')
    a.set(ylabel=yl, xlim=(0, d.t.iloc[-1]))
    if ttl: tag(a, ttl); a.set_xticklabels([])
    else: a.set_xlabel('Time [s]'); a.legend(loc='lower right', fontsize=5.6, handlelength=1.5)
dp = runs[PSRUN]['d']; dpc = dp[dp.t <= TCUT]; Mp = met(dp, tmax=TCUT)
NUM['point_persensor'] = dict(T_total=float(dp.t.iloc[-1]), tcut=TCUT, dist_to_goal_at_cut=float(np.hypot(dpc.kf_x.iloc[-1] + 1, dpc.kf_y.iloc[-1] + 1)), **Mp)
a2 = fig.add_subplot(gs[:, 2]); path(a2, dpc, '(c) Path, per-sensor $R$', PS, f"gap at {TCUT:g} s: {Mp['gap_end']*100:.0f} cm")
a2.legend(loc='upper left', bbox_to_anchor=(-0.02, 1.0), fontsize=5.6, handlelength=1.5, borderaxespad=0.1)
a3 = fig.add_subplot(gs[:, 3]); order = sorted(PT, key=lambda k: runs[k]['q'] / float(runs[k]['r'])); gu = []; gp = []
for k in order:
    dd = runs[k]['d']; gu.append(met(dd)['gap_end'] * 100); gp.append(met(dd, replay(dd, np.full(6, 0.25), R_PS)[0])['gap_end'] * 100)
    NUM['point_' + k] = dict(q=runs[k]['q'], r=float(runs[k]['r']), T=float(dd.t.iloc[-1]), gap_end_cm=gu[-1], gap_end_persensor_refiltered_cm=gp[-1], rms_v=met(dd)['rms_v'], rms_w=met(dd)['rms_w'])
xx = np.arange(len(order)); a3.bar(xx, gu, 0.62, color=EKF, label='uniform $Q$, $R$ (as run)')
a3.plot(xx, gp, 'D', ms=3.6, color=PS, mec='white', mew=0.5, ls='none', label='same log, per-sensor $R$ (re-filtered)')
for x0, g in zip(xx, gu): a3.text(x0, g + 0.8, f'{g:.0f}', ha='center', fontsize=5.8)
a3.set_xticks(xx); a3.set_xticklabels([f"{runs[k]['q']:g}\n{float(runs[k]['r']):g}" for k in order])
a3.set(ylabel='EKF-odometry gap at the goal [cm]', ylim=(0, 46)); a3.set_xlabel('$Q$ (top), $R$ (bottom); sorted by $Q/R$')
a3.grid(axis='x', visible=False); tag(a3, '(d) All 7 uniform point runs'); a3.legend(loc='upper left', fontsize=5.6, handlelength=1.2)
fig.savefig(OUT + '/fig_point.pdf'); plt.close(fig)
json.dump(NUM, open(OUT + '/numbers.json', 'w'), indent=1)
for k, v in NUM.items():
    if not k.startswith('median_'): print(k, json.dumps(v, default=float)[:330])
