from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union

# 폴리곤 리스트 예시
a = LineString([(0, 0), (1, 1)])
b = LineString([(0,1),(1, 0)])

if a.intersects(b):
    print(a.intersection(b))