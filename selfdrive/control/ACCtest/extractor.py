#!/usr/bin/env python3
import rospy, signal, sys
from novatel_oem7_msgs.msg import INSPVA
from threading import Lock
from pathlib import Path
import numpy as np, matplotlib, matplotlib.pyplot as plt
from pymap3d import geodetic2enu
matplotlib.use("Agg")

# ───── 조정 파라미터 ─────────────────────────────────────────
REF_LLH    = (37.38355121600099, 126.65694031645742, 30.76064212527126)
SPEED_JUMP = 5.0     # m/s   속도 스파이크 임계
WINDOW     = 5       # 이동 평균 창
TITLE      = "slow1"
FIGSIZE    = (12, 8)
DPI        = 500
# ────────────────────────────────────────────────────────────

class InspvaLoggerPlot:
    def __init__(self, title):
        self.title = title
        self.track = {}              # {ts: (lat, lon, h)}
        self.speed_log = []          # [(ts, speed_mag)]
        self.prev_v = 0.0
        self.lock = Lock()

        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.cb, queue_size=400)
        rospy.on_shutdown(self.save_and_plot)
        signal.signal(signal.SIGINT, self._sigint)

        self.res_dir = Path(__file__).resolve().parent / "result"
        self.res_dir.mkdir(exist_ok=True)

    # ───────── 콜백 ─────────────────────────────────────────
    def cb(self, msg):
        ts = msg.header.stamp.to_sec()
        lat, lon, h = msg.latitude, msg.longitude, msg.height
        # ENU speed
        v_mag = np.linalg.norm([msg.north_velocity,
                                msg.east_velocity,
                                msg.up_velocity])
        # if abs(v_mag - self.prev_v) > SPEED_JUMP:
        #     v_mag = self.prev_v
        self.prev_v = v_mag

        with self.lock:
            self.track[ts] = (lat, lon, h)
            self.speed_log.append((ts, v_mag))

    # ───────── 종료 처리 ────────────────────────────────────
    def _sigint(self, *_):
        rospy.loginfo("Ctrl+C → saving & plotting …")
        rospy.signal_shutdown("user interrupt")

    # ───────── 저장 + 플롯 ───────────────────────────────────
    def save_and_plot(self):
        txt_path = self.res_dir / "inspva_track.txt"

        with self.lock:
            pos_items   = sorted(self.track.items())
            speed_items = sorted(self.speed_log)

        # 1) 위치 TXT 저장
        with open(txt_path, "w") as f:
            for ts, (lat, lon, h) in pos_items:
                f.write(f"{ts:.6f} {lat:.10f} {lon:.10f} {h:.4f}\n")
        rospy.loginfo("track saved → %s", txt_path)

        # 2) 그래프들 작성
        self.make_figures(pos_items, speed_items)

    # ───────── 이동 평균 ────────────────────────────────────
    def _ma(self, x):
        return np.convolve(x, np.ones(WINDOW)/WINDOW, mode="same") if WINDOW>1 else x

    # ───────── 플롯 두 장 ───────────────────────────────────
    def make_figures(self, pos_items, speed_items):
        ts_pos = np.array([p[0] for p in pos_items])
        lats, lons, alts = map(np.asarray, zip(*(p[1] for p in pos_items)))
        e, n, u = geodetic2enu(lats, lons, alts, *REF_LLH)
        xyz = np.column_stack((e, n, u))

        ts_spd = np.array([s[0] for s in speed_items])
        v_raw  = np.array([s[1] for s in speed_items])
        # v_filt = self._ma(v_raw)
        v_filt = v_raw

        # 거리
        obs_e, obs_n, obs_u = geodetic2enu(*REF_LLH, *REF_LLH)
        dist = np.linalg.norm(xyz - np.array([obs_e, obs_n, obs_u]), axis=1)

        # 속도 시계열 보간
        v_interp = np.interp(ts_pos, ts_spd, v_filt)

        # 가속도 계산 + 이동 평균
        acc = np.zeros_like(v_interp)
        acc[1:] = np.diff(v_interp) / np.diff(ts_pos)
        # acc = self._ma(acc)

        mask = v_interp > 0.5
        ttc  = np.where(mask, dist / v_interp, np.nan)
        head = np.where(mask, dist / v_interp, np.nan)

        t_rel = ts_pos - ts_pos[0]

        # ── Figure 1 : Speed / Distance / TTC / Headway ──────
        self._plot_main(t_rel, v_interp, dist, ttc, head,
                        self.title,
                        self.res_dir / f"{self.title.replace(' ', '_')}.png")

        # ── Figure 2 : Acc / TTC / Headway ───────────────────
        self._plot_acc(t_rel, acc, ttc, head,
                       f"{self.title}_1",
                       self.res_dir / f"{self.title.replace(' ', '_')}_1.png")

    # ───────── 세부 플롯 함수들 ──────────────────────────────
    def _plot_main(self, t, v, dist, ttc, head, title, out_path):
        fig, ax1 = plt.subplots(figsize=FIGSIZE)
        ax1.plot(t, v, label="Speed (m/s)", c='tab:blue')
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Speed (m/s)", color='tab:blue')
        ax1.tick_params(axis='y', labelcolor='tab:blue')
        ax1.grid(axis='x', ls=':', lw=.6)

        ax2 = ax1.twinx()
        ax2.plot(t, dist, 'r--', label="Distance (m)")
        ax2.set_ylabel("Distance (m)", color='tab:red')
        ax2.tick_params(axis='y', labelcolor='tab:red')

        ax3 = ax1.twinx()
        ax3.spines['right'].set_position(("axes", 1.12))
        ax3.plot(t, ttc, 'g-.', label="TTC (s)")
        ax3.plot(t, head, 'purple', ls='dashdot', label="Headway (s)")
        ax3.set_ylabel("TTC / Headway (s)", color='tab:green')
        ax3.tick_params(axis='y', labelcolor='tab:green')

        lines, labels = [], []
        for ax in (ax1, ax2, ax3):
            l, lab = ax.get_legend_handles_labels()
            lines += l; labels += lab
        ax1.legend(lines, labels, loc='upper right')
        fig.suptitle(title); plt.tight_layout()
        plt.savefig(out_path, dpi=DPI); plt.close()
        rospy.loginfo("figure saved → %s", out_path)

    def _plot_acc(self, t, acc, ttc, head, title, out_path):
        fig, ax1 = plt.subplots(figsize=FIGSIZE)
        ax1.plot(t, acc, label="Acceleration (m/s²)", c='tab:orange')
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Acc (m/s²)", color='tab:orange')
        ax1.tick_params(axis='y', labelcolor='tab:orange')
        ax1.grid(axis='x', ls=':', lw=.6)

        ax2 = ax1.twinx()
        ax2.plot(t, ttc, 'g-.', label="TTC (s)")
        ax2.plot(t, head, 'purple', ls='dashdot', label="Headway (s)")
        ax2.set_ylabel("TTC / Headway (s)", color='tab:green')
        ax2.tick_params(axis='y', labelcolor='tab:green')

        lines, labels = [], []
        for ax in (ax1, ax2):
            l, lab = ax.get_legend_handles_labels()
            lines += l; labels += lab
        ax1.legend(lines, labels, loc='upper right')
        fig.suptitle(title); plt.tight_layout()
        plt.savefig(out_path, dpi=DPI); plt.close()
        rospy.loginfo("figure saved → %s", out_path)

# ───────── 메인 ────────────────────────────────────────────
if __name__ == "__main__":
    title = " ".join(sys.argv[1:]) or TITLE
    rospy.init_node("inspva_logger_plot", anonymous=False)
    InspvaLoggerPlot(title)
    rospy.loginfo("inspva_logger_plot running – Ctrl+C to save & plot")
    rospy.spin()
