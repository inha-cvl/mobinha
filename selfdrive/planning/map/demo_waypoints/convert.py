import pandas as pd
import numpy as np
import pymap3d as pm

# 입력 파일 경로와 파일명
input_file = "songdo-waypoint.csv"
# 출력 파일 경로와 파일명
output_file = "demo-waypoint-enu.csv"
# 기준 위도, 경도, 고도
base_lat, base_lon, base_alt = 37.3888319, 126.6428739, 7.369

# 입력 파일에서 데이터 읽기
df = pd.read_csv(input_file)

# 위도, 경도에서 ENU 좌표계 (e, n, u)로 변환하여 새로운 열에 추가
df['e'], df['n'], df['u'] = pm.geodetic2enu(df['Y'], df['X'], 7, 
                                              base_lat, base_lon, base_alt)

# 출력 파일에 쓰기
df.to_csv(output_file, index=False)
