import json

# JSON 파일을 불러오기
with open('KCity.json', 'r') as file:
    data = json.load(file)

# rightTurn이 True인 lanelet의 id를 저장하는 리스트
right_turn_lanelet_ids = []

# lanelets 탐색
for lanelet_id, lanelet_data in data['lanelets'].items():
    if lanelet_data.get('rightTurn', False):  # rightTurn이 True인 경우
        right_turn_lanelet_ids.append(lanelet_id)

# 결과 출력
for lanelet_id in right_turn_lanelet_ids:
    print(lanelet_id)
