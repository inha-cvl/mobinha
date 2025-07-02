#!/usr/bin/env python3
import sys, numpy as np, matplotlib, matplotlib.pyplot as plt
from pathlib import Path
from pymap3d import geodetic2enu

matplotlib.use("Agg")

MAX_JUMP = 5.0   # m/s  급격한 속도 변화 필터
WINDOW   = 5     # 이동 평균 창

def load_track(path):
    d = {}
    with open(path) as f:
        for line in f:
            ts, lat, lon, h = map(float, line.split())
            d[ts] = (lat, lon, h)
    return d

def moving_average(x, k):
    return np.convolve(x, np.ones(k) / k, mode="same") if k > 1 else x

def compute_speed(t_sec, xyz):
    v = np.zeros_like(t_sec)
    prev_v = 0.0
    for i in range(1, len(t_sec)):
        dt = t_sec[i] - t_sec[i - 1]
        ds = np.linalg.norm(xyz[i] - xyz[i - 1])
        cur_v = ds / dt
        if abs(cur_v - prev_v) > MAX_JUMP:
            cur_v = prev_v
        v[i] = cur_v
        prev_v = cur_v
    return moving_average(v, WINDOW)

def plot_speed_dist_ttc_headway(
    veh_log,
    obstacle_llh,
    title,
    ref_llh=(37.38355121600099, 126.65694031645742, 30.76064212527126),
):
    ts = np.array(sorted(veh_log.keys()), dtype=float)
    lats, lons, alts = map(np.asarray, zip(*(veh_log[t] for t in ts)))
    e, n, u = geodetic2enu(lats, lons, alts, *ref_llh)
    xyz = np.column_stack((e, n, u))

    speed = compute_speed(ts, xyz)  # [m/s]

    obs_e, obs_n, obs_u = geodetic2enu(*obstacle_llh, *ref_llh)
    dist = np.linalg.norm(xyz - np.array([obs_e, obs_n, obs_u]), axis=1)  # [m]

    # 속도가 너무 작으면 NaN 처리
    mask = speed > 0.5
    ttc = np.where(mask, dist / speed, np.nan)        # [s]
    headway = np.where(mask, dist / speed, np.nan)    # (정의 같음)

    # ────────── 플롯 ────────────────────────────────
    t_rel = ts - ts[0]
    fig, ax1 = plt.subplots(figsize=(9, 4.8))

    ax1.plot(t_rel, speed, label="Speed (m/s)", color="tab:blue")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Speed (m/s)", color="tab:blue")
    ax1.tick_params(axis="y", labelcolor="tab:blue")
    ax1.grid(axis="x", linestyle=":", linewidth=0.6)

    ax2 = ax1.twinx()
    ax2.plot(t_rel, dist, "r--", label="Distance (m)")
    ax2.set_ylabel("Distance (m)", color="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:red")

    ax3 = ax1.twinx()
    ax3.spines.right.set_position(("axes", 1.12))           # 축 약간 우측 이동
    ax3.plot(t_rel, ttc, "g-.", label="TTC (s)")
    ax3.plot(t_rel, headway, "purple", linestyle="dashdot", label="Headway (s)")
    ax3.set_ylabel("TTC / Headway (s)", color="tab:green")
    ax3.tick_params(axis="y", labelcolor="tab:green")

    # 레전드 병합
    lines, labels = [], []
    for ax in (ax1, ax2, ax3):
        l, lab = ax.get_legend_handles_labels()
        lines += l
        labels += lab
    ax1.legend(lines, labels, loc="upper right")

    fig.suptitle(title)
    plt.tight_layout()

    out_dir = Path(__file__).resolve().parent / "result"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / f"{title.replace(' ', '_')}.png"
    plt.savefig(out_path, dpi=400)
    plt.close()
    print(f"saved {out_path}")

# ────────── 실행 ─────────────────────────────────
if __name__ == "__main__":
    base = Path(__file__).resolve().parent
    veh_log = load_track(base / "inspva_track.txt")
    obstacle = (37.38355121600099, 126.65694031645742, 30.76064212527126)
    title = " ".join(sys.argv[1:]) or "fast2"
    plot_speed_dist_ttc_headway(veh_log, obstacle, title)
