import os
import re
import pandas as pd
import numpy as np
from scipy.interpolate import LinearNDInterpolator
from pathlib import Path

class PedalMapper:
    def __init__(self):
        data_dir = Path(__file__).parent / "speed_thlbrk_accel_csv"
        csv_pattern = re.compile(r"speed_.*_accel\.csv", re.IGNORECASE)

        dfs = []
        for fn in os.listdir(data_dir):     
            if csv_pattern.match(fn):
                path = os.path.join(data_dir, fn)
                df_tmp = pd.read_csv(path)
                df_tmp.columns = [c.strip().lower() for c in df_tmp.columns]
                dfs.append(df_tmp[['speed', 'throttle', 'accel']])

        if not dfs:
            raise RuntimeError("❗ 변환된 CSV 파일을 찾지 못했습니다. '/mnt/data' 경로에 speed_*_accel.csv 파일이 있어야 합니다.")

        self.df = pd.concat(dfs, ignore_index=True)
        points = self.df[["speed", "accel"]].to_numpy()
        values = self.df["throttle"].to_numpy()
        self._interp = LinearNDInterpolator(points, values, fill_value=np.nan)

    def _nearest(self, spd: float, acc: float) -> float:
        diffs = np.hypot(self.df["speed"] - spd, self.df["accel"] - acc)
        idx = diffs.idxmin()
        return float(self.df.loc[idx, "throttle"])

    def command(self, speed: float, accel_target: float) -> float:
        cmd = self._interp(speed, accel_target)
        if np.isnan(cmd):
            cmd = self._nearest(speed, accel_target)

        if accel_target < 0 and cmd > 0:
            cmd = -cmd
        return float(cmd)

if __name__ == "__main__":
    mapper = PedalMapper()

    examples = [
        # (5.0,  0.4),   # 가속
        # (5.0, -0.5),   # 감속
        # (12.0, 0.0),   # 유지
        # (11.5, -0.6),
        # (0.8, 3.0),
        # (5.0, -0.32)
        (1.0, 0.5),
        (3.0, 0.5),
        (6.0, 0.5),
        (9.0, 0.5),
        (12.0, 0.5),
        (15.0, 0.5),
    ]

    print("예시 결과:")
    for spd, acc in examples:
        pedal = mapper.command(spd, acc)
        print(f" speed={spd:5.1f} m/s, accel_target={acc:+5.2f} m/s² -> pedal={pedal:+.2f} %")
