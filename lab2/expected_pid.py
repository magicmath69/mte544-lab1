import numpy as np

TAU = 0.2
KLP, KLV, KLI = 0.2, 0.05, 0.1
KAP, KAV, KAI = 0.5, 0.25, 0.03


def run(kp, kd, ki, e0, dt=0.001, T=60):
    e, z, u = e0, 0.0, 0.0
    rows = []
    for i in range(int(T / dt)):
        cmd = kp * e - kd * u + ki * z
        u += (cmd - u) * dt / TAU
        rows.append((i * dt, e / e0, u))
        e -= u * dt
        z += e * dt
    return np.array(rows)


def show(name, kp, kd, ki, e0):
    t, n, u = run(kp, kd, ki, e0).T
    rise = t[np.argmax(n <= 0.1)] - t[np.argmax(n <= 0.9)]
    settle = t[np.where(abs(n) > 0.05)[0][-1]]
    overshoot = max(0, -n.min()) * 100
    line = f"{name:14s} rise {rise:5.2f} s  settle 5% {settle:5.1f} s  overshoot {overshoot:4.1f} %"
    if (n <= 0).any():
        i = np.argmax(n <= 0)
        line += f"  zero at {t[i]:4.2f} s (rate {u[i]:.3f})  peak at {t[np.argmin(n)]:4.1f} s"
    print(line)


def zeta(kp, kd, ki):
    return kp / (2 * np.sqrt((1 + kd) * ki))


def zeta_lag(kp, kd):
    return (1 + kd) / (2 * np.sqrt(kp * TAU))


print("distance loop, start 1 m")
show("P", KLP, 0, 0, 1.0)
show("PID", KLP, KLV, KLI, 1.0)
print("heading loop, start 180 deg")
show("P", KAP, 0, 0, np.pi)
show("PD", KAP, KAV, 0, np.pi)
show("PID", KAP, KAV, KAI, np.pi)
print()
print(f"distance PID zeta {zeta(KLP, KLV, KLI):.2f}, kli for zeta 0.7: {KLP**2 / (4 * 0.49 * (1 + KLV)):.3f}")
print(f"heading PID roots {np.roots([1 + KAV, KAP, KAI]).round(3)}, slow time constant {-1 / np.roots([1 + KAV, KAP, KAI]).max():.1f} s")
print(f"heading P zeta: kap 0.4 {zeta_lag(0.4, 0):.2f}, kap 0.5 {zeta_lag(0.5, 0):.2f}, PD {zeta_lag(0.5, KAV):.2f}")
z = zeta_lag(2.5, 0)
print(f"heading P kap 2.5: zeta {z:.2f}, overshoot {100 * np.exp(-np.pi * z / np.sqrt(1 - z**2)):.1f} %")
