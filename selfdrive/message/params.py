from selfdrive.message.car_message import car_param

import json
import os

dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def getBaseLLA(map_path):
    with open(map_path) as f:
        data = json.load(f)
        base = data['base_lla']
        return base[0], base[1], base[2]


def getPrecision(map_path):
    with open(map_path) as f:
        data = json.load(f)
        precision = data['precision']
    return precision


class IONIQ:
    def __init__(self, map_name='KCity'):
        self.CP = car_param.CarParam()
        car_param_dict = self.CP._asdict()
        car_param_dict["name"] = 'ioniq'
        car_param_dict["dbc"] = dir_path + \
            '/car/dbc/%s/can.dbc' % car_param_dict["name"]
        car_param_map_param = car_param_dict["mapParam"]._asdict()
        car_param_map_param["path"] = dir_path + \
            '/planning/map/%s.json' % map_name
        car_param_map_param["precision"] = getPrecision(car_param_map_param["path"])
        base_lat, base_lng, base_alt = getBaseLLA(car_param_map_param["path"])
        car_param_map_param["baseLatitude"] = base_lat
        car_param_map_param["baseLongitude"] = base_lng
        car_param_map_param["baseAltitude"] = base_alt
        car_param_dict["mapParam"] = self.CP.mapParam._make(
            car_param_map_param.values())

        car_param_dict["minEnableSpeed"] = 10  # songdo demo site: 21, test site: 15 # min_v
        car_param_dict["maxEnableSpeed"] = 44  #45 # ref_v
        car_param_dict["mass"] = 1737.0 + 136.0
        car_param_dict["wheelbase"] = 3.00  # L
        car_param_dict["centerToFront"] = car_param_dict["wheelbase"] * 0.4
        car_param_dict["steerRatio"] = 13.5

        car_param_longitudinal_tuning = car_param_dict["longitudinalTuning"]._asdict(
        )
        car_param_longitudinal_tuning["kpV"] = 2.98  # K_P
        car_param_longitudinal_tuning["kiV"] = 2.38  # K_I
        car_param_longitudinal_tuning["kf"] = 0.00  # K_D
        car_param_dict["longitudinalTuning"] = self.CP.longitudinalTuning._make(
            car_param_longitudinal_tuning.values())

        car_param_lateral_tuning = car_param_dict["lateralTuning"]._asdict()
        car_param_lateral_tuning_lqr = car_param_lateral_tuning["lqr"]._asdict(
        )
        car_param_lateral_tuning_lqr["l"] = 1.0  # Lfc, look-ahead distance
        car_param_lateral_tuning_lqr["k"] = 1.5 # origin 1.5 lane change good is 2  # k, look forward gain
        car_param_lateral_tuning["lqr"] = self.CP.lateralTuning.lqr._make(
            car_param_lateral_tuning_lqr.values())
        car_param_dict["lateralTuning"] = self.CP.lateralTuning._make(
            car_param_lateral_tuning.values())

        car_param_dict["vEgoStarting"] = 0.1
        car_param_dict["startAccel"] = 2.0

        self.CP = self.CP._make(car_param_dict.values())


class SIMULATOR:
    def __init__(self, map_name='KCity'):
        self.CP = car_param.CarParam()
        car_param_dict = self.CP._asdict()
        car_param_dict["name"] = 'simulator'
        car_param_dict["dbc"] = dir_path + \
            '/car/dbc/%s/can.dbc' % car_param_dict["name"]
        car_param_map_param = car_param_dict["mapParam"]._asdict()
        car_param_map_param["path"] = dir_path + \
            '/planning/map/%s.json' % map_name
        car_param_map_param["precision"] = getPrecision(car_param_map_param["path"])
        base_lat, base_lng, base_alt = getBaseLLA(car_param_map_param["path"])
        car_param_map_param["baseLatitude"] = base_lat
        car_param_map_param["baseLongitude"] = base_lng
        car_param_map_param["baseAltitude"] = base_alt
        car_param_dict["mapParam"] = self.CP.mapParam._make(
            car_param_map_param.values())

        car_param_dict["minEnableSpeed"] = 23  # min_v
        car_param_dict["maxEnableSpeed"] = 45  # ref_v

        car_param_dict["mass"] = 1737.0 + 136.0
        car_param_dict["wheelbase"] = 2.72  # L
        car_param_dict["centerToFront"] = car_param_dict["wheelbase"] * 0.4
        car_param_dict["steerRatio"] = 1

        car_param_longitudinal_tuning = car_param_dict["longitudinalTuning"]._asdict(
        )
        car_param_longitudinal_tuning["kpV"] = 2.98   # K_P
        car_param_longitudinal_tuning["kiV"] = 2.38  # K_I
        car_param_longitudinal_tuning["kf"] = 0.0  # K_D
        car_param_dict["longitudinalTuning"] = self.CP.longitudinalTuning._make(
            car_param_longitudinal_tuning.values())

        car_param_lateral_tuning = car_param_dict["lateralTuning"]._asdict()
        car_param_lateral_tuning_lqr = car_param_lateral_tuning["lqr"]._asdict(
        )
        car_param_lateral_tuning_lqr["l"] = 1.5  # Lfc, look-ahead distance
        car_param_lateral_tuning_lqr["k"] = 1.5  # k, look forward gain
        car_param_lateral_tuning["lqr"] = self.CP.lateralTuning.lqr._make(
            car_param_lateral_tuning_lqr.values())
        car_param_dict["lateralTuning"] = self.CP.lateralTuning._make(
            car_param_lateral_tuning.values())

        car_param_dict["vEgoStarting"] = 0.1
        car_param_dict["startAccel"] = 2.0

        self.CP = self.CP._make(car_param_dict.values())


class MORAI: # HYUNDAI NEXO
    def __init__(self, map_name='KCity'):
        self.CP = car_param.CarParam()
        car_param_dict = self.CP._asdict()
        car_param_dict["name"] = 'morai'
        car_param_dict["dbc"] = dir_path + \
            '/car/dbc/%s/can.dbc' % car_param_dict["name"]
        car_param_map_param = car_param_dict["mapParam"]._asdict()
        car_param_map_param["path"] = dir_path + \
            '/planning/map/%s.json' % map_name
        car_param_map_param["precision"] = getPrecision(car_param_map_param["path"])
        base_lat, base_lng, base_alt = getBaseLLA(car_param_map_param["path"])
        car_param_map_param["baseLatitude"] = base_lat
        car_param_map_param["baseLongitude"] = base_lng
        car_param_map_param["baseAltitude"] = base_alt
        car_param_dict["mapParam"] = self.CP.mapParam._make(
            car_param_map_param.values())

        car_param_dict["minEnableSpeed"] = 15  # min_v
        car_param_dict["maxEnableSpeed"] = 50  # ref_v
        car_param_dict["mass"] = 1840 + 140 # unladen weight + 1 driver and 1 passenger 
        car_param_dict["wheelbase"] = 2.79  # L, vehicle length 
        car_param_dict["centerToFront"] = car_param_dict["wheelbase"] * 0.4
        car_param_dict["steerRatio"] = 1

        car_param_longitudinal_tuning = car_param_dict["longitudinalTuning"]._asdict(
        )
        car_param_longitudinal_tuning["kpV"] = 8.5   # K_P
        car_param_longitudinal_tuning["kiV"] = 6.0  # K_I
        car_param_longitudinal_tuning["kf"] = 2.25 # K_D
        car_param_dict["longitudinalTuning"] = self.CP.longitudinalTuning._make(
            car_param_longitudinal_tuning.values())

        car_param_lateral_tuning = car_param_dict["lateralTuning"]._asdict()
        car_param_lateral_tuning_lqr = car_param_lateral_tuning["lqr"]._asdict(
        )
        car_param_lateral_tuning_lqr["l"] = 1.0  # Lfc, look-ahead distance
        car_param_lateral_tuning_lqr["k"] = 1.5  # k, look forward gain
        car_param_lateral_tuning["lqr"] = self.CP.lateralTuning.lqr._make(
            car_param_lateral_tuning_lqr.values())
        car_param_dict["lateralTuning"] = self.CP.lateralTuning._make(
            car_param_lateral_tuning.values())

        car_param_dict["vEgoStarting"] = 0.1
        car_param_dict["startAccel"] = 2.0

        self.CP = self.CP._make(car_param_dict.values())
