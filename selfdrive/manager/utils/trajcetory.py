import json
import pandas as pd
import pymap3d as pm

# CSV 파일 읽기
df = pd.read_csv("../data/trajectory.csv")

# Base LLA 좌표
base_lla = [37.3888319, 126.6428739, 7.369]

# 빈 list 생성
enu_coords = []

# 위경도를 ENU로 변환
for index, row in df.iterrows():
    lat, lon = row['field.latitude'], row['field.longitude']
    x, y, _ = pm.geodetic2enu(lat, lon, 0, base_lla[0], base_lla[1], base_lla[2])
    enu_coords.append([x, y])

# JSON 데이터 생성
json_data = {
    "trajectory": enu_coords
}

# JSON 파일에 저장
with open("../data/trajectory.json", "w") as f:
    json.dump(json_data, f)