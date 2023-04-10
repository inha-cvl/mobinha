
import numpy as np
import pymap3d as pm


base_lat, base_lon, base_alt = 37.3888319, 126.6428739, 7.369

lat, lon = 37.390070, 126.644268




# 위도, 경도에서 ENU 좌표계 (e, n, u)로 변환하여 새로운 열에 추가
e, n, u = pm.geodetic2enu(lat,lon,7, base_lat, base_lon, base_alt)

print(e,n,u)
