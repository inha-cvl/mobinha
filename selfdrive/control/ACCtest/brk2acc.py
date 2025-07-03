#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
        self.base_time = None                     

        # 버퍼
        self.acc_t, self.acc_norms = [], []     
        self.vel_t, self.vel_norms = [], []   

        rospy.Subscriber("/imu/data_raw", Imu, self.imu_cb,     queue_size=200)
        rospy.Subscriber("/novatel/oem7/inspva", INSPVA, self.inspva_cb, queue_size=200)
        rospy.on_shutdown(self.save_plot)

        self.prev_v = None
        self.decel_eps = rospy.get_param("~decel_eps", 0.05)
        self.trigger = False

    def imu_cb(self, msg: Imu):
        if self.base_time is None:             
            return
        t = msg.header.stamp.to_sec() - self.base_time
        a_norm = np.linalg.norm([msg.linear_acceleration.x,
                                 msg.linear_acceleration.y,
                                 msg.linear_acceleration.z]) / 10.0   
        self.acc_t.append(t)
        self.acc_norms.append(a_norm)

    def inspva_cb(self, msg: INSPVA):
        stamp = msg.header.stamp.to_sec()

        if self.base_time is None:
            self.base_time = stamp
            rospy.loginfo(f"Base time set to {self.base_time:.3f} s")

        t = stamp - self.base_time

        if not (14<t<40):
            return
        
        # 속도 크기 계산
        v_norm = np.linalg.norm([
            msg.north_velocity,
            msg.east_velocity,
            msg.up_velocity
        ])

        # 버퍼 저장
        self.vel_t.append(t)
        self.vel_norms.append(v_norm)

    def save_plot(self):
        if not self.vel_t or not self.acc_t:
            rospy.logwarn("No data collected; plot skipped.")
            return

        acc_t  = np.asarray(self.acc_t)
        acc_n  = np.asarray(self.acc_norms)
        vel_t  = np.asarray(self.vel_t)
        vel_n  = np.asarray(self.vel_norms)

        t0 = max(acc_t.min(), vel_t.min())    
        t1 = min(acc_t.max(), vel_t.max())     
        if t1 - t0 < 0.2:                     
            rospy.logwarn("Overlap too short; plot skipped.")
            return

        dt = 0.1                             
        t_u = np.arange(t0, t1, dt)           
        vel_u = np.interp(t_u, vel_t, vel_n)  
        acc_u = np.interp(t_u, acc_t, acc_n)   

        mask = (0.1 < vel_u) & (vel_u < 49.0/3.6)
        if not np.any(mask):
            rospy.logwarn("No samples in 0-50/3.6 m/s range; plot skipped.")
            return
        t_sel   = t_u[mask]
        vel_sel = vel_u[mask]
        acc_sel = acc_u[mask]

        dv_dt = np.gradient(vel_sel, dt)
        
        _brk = 0
        df = pd.DataFrame({
            "speed":    vel_sel,
            "throttle": _brk,   
            "accel":    dv_dt
        })
        _brk_str = str(_brk)
        png_name = "brk"+_brk_str+".png"
        xlsx_name = "speed_throttle_accel_brk"+_brk_str+".xlsx"
        xlsx_path = Path(rospy.get_param("~xlsx", xlsx_name)).expanduser()
        df.to_excel(xlsx_path, index=False)
        rospy.loginfo(f"Data exported → {xlsx_path}")


        plt.figure(figsize=(12, 6))
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
