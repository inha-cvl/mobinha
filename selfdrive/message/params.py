from selfdrive.message.car_message import car_param

import json
import os

MAP = 'KCity'  # 'songdo'

dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def getBaseLLA(map_path):
    with open(map_path) as f:
        data = json.load(f)
        base = data['base_lla']
        return base[0], base[1], base[2]


class NIRO:
    CP = car_param.CarParam()
    car_param = CP._asdict()
    car_param["name"] = 'niro'
    car_param["dbc"] = dir_path+'/car/dbc/%s/can.dbc' % car_param["name"]
    car_param_map_param = car_param["mapParam"]._asdict()
    car_param_map_param["path"] = dir_path+'/planning/map/%s.json' % MAP
    car_param_map_param["tileSize"] = 5.0
    car_param_map_param["cutDist"] = 15.0
    car_param_map_param["precision"] = 0.5
    base_lat, base_lng, base_alt = getBaseLLA(car_param_map_param["path"])
    car_param_map_param["baseLatitude"] = base_lat
    car_param_map_param["baseLongitude"] = base_lng
    car_param_map_param["baseAltitude"] = base_alt
    car_param["mapParam"] = CP.mapParam._make(car_param_map_param.values())

    car_param["minEnableSpeed"] = 3  # min_v
    car_param["maxEnableSpeed"] = 50  # ref_v
    car_param["mass"] = 1737.0 + 136.0
    car_param["wheelbase"] = 2.72  # L
    car_param["centerToFront"] = car_param["wheelbase"] * 0.4
    car_param["steerRatio"] = 13.73

    car_param_longitudinal_tuning = car_param["longitudinalTuning"]._asdict()
    car_param_longitudinal_tuning["kpV"] = 0.75  # K_P
    car_param_longitudinal_tuning["kiV"] = 0.15  # K_I
    car_param_longitudinal_tuning["kf"] = 0.06  # K_D
    car_param["longitudinalTuning"] = CP.longitudinalTuning._make(
        car_param_longitudinal_tuning.values())

    car_param_lateral_tuning = car_param["lateralTuning"]._asdict()
    car_param_lateral_tuning_lqr = car_param_lateral_tuning["lqr"]._asdict()
    car_param_lateral_tuning_lqr["l"] = 3.0  # Lfc, look-ahead distance
    car_param_lateral_tuning_lqr["k"] = 0.85  # k, look forward gain
    car_param_lateral_tuning["lqr"] = CP.lateralTuning.lqr._make(
        car_param_lateral_tuning_lqr.values())
    car_param["lateralTuning"] = CP.lateralTuning._make(
        car_param_lateral_tuning.values())

    car_param["vEgoStarting"] = 0.1
    car_param["startAccel"] = 2.0

    CP = CP._make(car_param.values())


class SIMULATOR:
    CP = car_param.CarParam()
    car_param = CP._asdict()
    car_param["name"] = 'simulator'
    car_param["dbc"] = dir_path+'/car/dbc/%s/can.dbc' % car_param["name"]
    car_param_map_param = car_param["mapParam"]._asdict()
    car_param_map_param["path"] = dir_path+'/planning/map/%s.json' % MAP
    car_param_map_param["tileSize"] = 5.0
    car_param_map_param["cutDist"] = 15.0
    car_param_map_param["precision"] = 0.5

    base_lat, base_lng, base_alt = getBaseLLA(car_param_map_param["path"])
    car_param_map_param["baseLatitude"] = base_lat
    car_param_map_param["baseLongitude"] = base_lng
    car_param_map_param["baseAltitude"] = base_alt
    car_param["mapParam"] = CP.mapParam._make(car_param_map_param.values())

    car_param["minEnableSpeed"] = 10  # min_v
    car_param["maxEnableSpeed"] = 50  # ref_v
    car_param["mass"] = 1737.0 + 136.0
    car_param["wheelbase"] = 2.72  # L
    car_param["centerToFront"] = car_param["wheelbase"] * 0.4
    car_param["steerRatio"] = 13.73

    car_param_longitudinal_tuning = car_param["longitudinalTuning"]._asdict()
    car_param_longitudinal_tuning["kpV"] = 0.75  # K_P
    car_param_longitudinal_tuning["kiV"] = 0.15  # K_I
    car_param_longitudinal_tuning["kf"] = 0.06  # K_D
    car_param["longitudinalTuning"] = CP.longitudinalTuning._make(
        car_param_longitudinal_tuning.values())

    car_param_lateral_tuning = car_param["lateralTuning"]._asdict()
    car_param_lateral_tuning_lqr = car_param_lateral_tuning["lqr"]._asdict()
    car_param_lateral_tuning_lqr["l"] = 3.0  # Lfc, look-ahead distance
    car_param_lateral_tuning_lqr["k"] = 0.85  # k, look forward gain
    car_param_lateral_tuning["lqr"] = CP.lateralTuning.lqr._make(
        car_param_lateral_tuning_lqr.values())
    car_param["lateralTuning"] = CP.lateralTuning._make(
        car_param_lateral_tuning.values())

    car_param["vEgoStarting"] = 0.1
    car_param["startAccel"] = 2.0

    CP = CP._make(car_param.values())


class MORAI:
    CP = car_param.CarParam()
    car_param = CP._asdict()
    car_param["name"] = 'morai'
    car_param["dbc"] = dir_path+'/car/dbc/%s/can.dbc' % car_param["name"]
    car_param_map_param = car_param["mapParam"]._asdict()
    car_param_map_param["path"] = dir_path+'/planning/map/%s.json' % MAP
    car_param_map_param["tileSize"] = 5.0
    car_param_map_param["cutDist"] = 15.0
    car_param_map_param["precision"] = 0.5

    base_lat, base_lng, base_alt = getBaseLLA(car_param_map_param["path"])
    car_param_map_param["baseLatitude"] = base_lat
    car_param_map_param["baseLongitude"] = base_lng
    car_param_map_param["baseAltitude"] = base_alt
    car_param["mapParam"] = CP.mapParam._make(car_param_map_param.values())

    car_param["minEnableSpeed"] = 30  # min_v
    car_param["maxEnableSpeed"] = 80  # ref_v
    car_param["mass"] = 1737.0 + 136.0
    car_param["wheelbase"] = 2.8  # L, vehicle length
    car_param["centerToFront"] = car_param["wheelbase"] * 0.4
    car_param["steerRatio"] = 13.73

    car_param_longitudinal_tuning = car_param["longitudinalTuning"]._asdict()
    car_param_longitudinal_tuning["kpV"] = 0.1  # K_P
    car_param_longitudinal_tuning["kiV"] = 0.0  # K_I
    car_param_longitudinal_tuning["kf"] = 0.05  # K_D
    car_param["longitudinalTuning"] = CP.longitudinalTuning._make(
        car_param_longitudinal_tuning.values())

    car_param_lateral_tuning = car_param["lateralTuning"]._asdict()
    car_param_lateral_tuning_lqr = car_param_lateral_tuning["lqr"]._asdict()
    car_param_lateral_tuning_lqr["l"] = 5.0  # Lfc, look-ahead distance
    car_param_lateral_tuning_lqr["k"] = 0.65  # k, look forward gain
    car_param_lateral_tuning["lqr"] = CP.lateralTuning.lqr._make(
        car_param_lateral_tuning_lqr.values())
    car_param["lateralTuning"] = CP.lateralTuning._make(
        car_param_lateral_tuning.values())

    car_param["vEgoStarting"] = 0.1
    car_param["startAccel"] = 2.0

    CP = CP._make(car_param.values())
