import json

# JSON 파일을 열고, 파일 내용을 Python dict로 변환
with open('/home/jourmain/mobinha/selfdrive/planning/map/KCity.json', 'r') as file:
    dict_data = json.load(file)

# for i in range(len(dict_data["lanelets"])):
#     # if len(dict_data["lanelets"][f"{i}"]["successor"]) == 0:
#     if len(dict_data["lanelets"]["91"]["successor"]) == 0:
#         print(i)
# print(dict_data["lanelets"]["429"]["adjacentRight"])
# print(dict_data["lanelets"]["429"]["adjacentLeft"])
# print(dict_data["lanelets"]["429"]["predecessor"])
# print(dict_data["surfacemarks"]["B319BS010001"])

polygon_points = []
crosswalk_ids = dict_data["lanelets"]["643"]['crosswalkID']
for s_id in crosswalk_ids:
    polygon_points.append((s_id, dict_data["surfacemarks"][s_id])) # points [], [] ....
    print("crosswalk polygon points", polygon_points)

