import math

def calculate_distance(cs_position, stopline_point1, stopline_point2, yaw, distance_to_stopline):
    # 정지선의 중점 계산
    stopline_midpoint = ((stopline_point1[0] + stopline_point2[0]) / 2, 
                         (stopline_point1[1] + stopline_point2[1]) / 2)

    # 차량의 진행 방향 벡터 계산 (yaw를 이용)
    heading_vector = (math.cos(yaw), math.sin(yaw))

    # 차량 위치에서 정지선 중점까지의 벡터 계산
    ego_to_stopline = (stopline_midpoint[0] - cs_position[0], stopline_midpoint[1] - cs_position[1])

    # 도트 곱 계산
    dot_product_value = sum(a * b for a, b in zip(ego_to_stopline, heading_vector))

    # 차량이 정지선을 지났으면 거리 값을 음수로 변환
    if dot_product_value < 0:
        distance_to_stopline = -abs(distance_to_stopline)

    return distance_to_stopline

# 예시 데이터
cs_position = (-6.625972010228452, 1820.4373979690063)
stopline_point1 = (-5.04056351618874, 1827.5972518328967)
stopline_point2 = (-8.869905981685633, 1827.4941411976429)
yaw = 0.1  # 예시로 사용한 yaw 값
distance_to_stopline = 7.1145857343398  # 예시 거리 값

# 거리 계산
new_distance = calculate_distance(cs_position, stopline_point1, stopline_point2, yaw, distance_to_stopline)
print(f"Calculated Distance to Stopline: {new_distance}")
