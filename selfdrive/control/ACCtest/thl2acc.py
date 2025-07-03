#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IMU |a| (scaled 1/10), INSPVA |v|, d|v|/dt (0.1 s 샘플) 플롯
저장: imu_inspva_plot.png
ROS Noetic, Python ≥3.8
"""

import rospy
from sensor_msgs.msg import Imu
from novatel_oem7_msgs.msg import INSPVA

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd   


class ImuInspvaCollector:
    def __init__(self):
        self.base_time = None                      # 첫 INSPVA 기준 시각

        # 버퍼
        self.acc_t, self.acc_norms = [], []        # IMU
        self.vel_t, self.vel_norms = [], []        # INSPVA

        rospy.Subscriber("/imu/data_raw", Imu, self.imu_cb,     queue_size=200)
        rospy.Subscriber("/novatel/oem7/inspva", INSPVA, self.inspva_cb, queue_size=200)
        rospy.on_shutdown(self.save_plot)

        self.trigger = False

    # ------------------------- 콜백 -------------------------
    def imu_cb(self, msg: Imu):
        if self.base_time is None:             # 기준 시각 미설정
            return
        t = msg.header.stamp.to_sec() - self.base_time
        a_norm = np.linalg.norm([msg.linear_acceleration.x,
                                 msg.linear_acceleration.y,
                                 msg.linear_acceleration.z]) / 10.0   # 단위 보정
        self.acc_t.append(t)
        self.acc_norms.append(a_norm)

    def inspva_cb(self, msg: INSPVA):
        stamp = msg.header.stamp.to_sec()

        # 기준 시각 설정
        if self.base_time is None:
            self.base_time = stamp
            rospy.loginfo(f"Base time set to {self.base_time:.3f} s")

        t = stamp - self.base_time

        # 속도 크기
        v_norm = np.linalg.norm([
            msg.north_velocity,
            msg.east_velocity,
            msg.up_velocity
        ])

        # -------- 슬라이싱: 0 < v < 50 m/s 범위가 아니면 무시 --------
        if v_norm < 0.1:
            return
        if v_norm > 48/3.6:
        # if v_norm > 8.1:
            self.trigger = True
        if self.trigger:
            return

        # 범위에 들어오는 데이터만 저장
        self.vel_t.append(t)
        self.vel_norms.append(v_norm)


    # ------------------------- 종료 시 플롯 -------------------------
    def save_plot(self):
        # ------------------ 0. 데이터 존재 확인 ------------------
        if not self.vel_t or not self.acc_t:
            rospy.logwarn("No data collected; plot skipped.")
            return

        # ------------------ 1. 배열 변환 ------------------
        acc_t  = np.asarray(self.acc_t)
        acc_n  = np.asarray(self.acc_norms)
        vel_t  = np.asarray(self.vel_t)
        vel_n  = np.asarray(self.vel_norms)

        # ------------------ 2. 공통 시간구간 계산 ------------------
        t0 = max(acc_t.min(), vel_t.min())       # 겹치는 구간 시작
        t1 = min(acc_t.max(), vel_t.max())       # 겹치는 구간 끝
        if t1 - t0 < 0.2:                        # 0.2 s 미만이면 의미 없음
            rospy.logwarn("Overlap too short; plot skipped.")
            return

        # ------------------ 3. 균일 시간축 & 보간 ------------------
        dt = 0.1                                 # 0.1 s 간격
        t_u = np.arange(t0, t1, dt)              # 공통 격자
        vel_u = np.interp(t_u, vel_t, vel_n)     # |v|(t)
        acc_u = np.interp(t_u, acc_t, acc_n)     # |a|(t)

        # ------------------ 4. 속도 조건 마스킹 ------------------
        mask = (0.1 < vel_u) & (vel_u < 49.0/3.6)
        if not np.any(mask):
            rospy.logwarn("No samples in 0-50/3.6 m/s range; plot skipped.")
            return
        t_sel   = t_u[mask]
        vel_sel = vel_u[mask]
        acc_sel = acc_u[mask]

        # d|v|/dt (수치 미분)
        dv_dt = np.gradient(vel_sel, dt)
        
        _thl = 10
        df = pd.DataFrame({
            "speed":    vel_sel,
            "throttle": _thl,   
            "accel":    dv_dt
        })
        _thl_str = str(_thl)
        png_name = "thl"+_thl_str+".png"
        xlsx_name = "speed_throttle_accel_thl"+_thl_str+".xlsx"
        xlsx_path = Path(rospy.get_param("~xlsx", xlsx_name)).expanduser()
        df.to_excel(xlsx_path, index=False)
        rospy.loginfo(f"Data exported → {xlsx_path}")


        # ------------------ 5. 플롯 ------------------
        plt.figure(figsize=(12, 6))
        # plt.plot(t_sel, acc_sel, label="|a| (IMU, ÷10)")
        plt.plot(t_sel, vel_sel, label="|v| (INSPVA)")
        plt.plot(t_sel, dv_dt,   label="d|v|/dt (num diff)")

        plt.xlabel("timestamp [s]")
        plt.ylabel("value")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        fname = Path(rospy.get_param("~output", png_name)).expanduser()
        plt.savefig(fname, dpi=150)
        rospy.loginfo(f"Figure saved → {fname}")

if __name__ == "__main__":
    rospy.init_node("imu_inspva_collector", anonymous=False)
    ImuInspvaCollector()
    rospy.loginfo("Collecting …  Press Ctrl+C to stop and save.")
    rospy.spin()
