from message.messaging import CP
# all speeds in m/s


class NIRO:
    CP = CP
    car_param = CP._asdict()
    car_param["name"] = 'niro'
    car_param["dbc"] = 'dbc/niro/can.dbc'
    car_param_map_param = car_param["mapParam"]._asdict()
    car_param_map_param["path"] = 'map/songdo.json'
    car_param_map_param["tileSize"] = 5.0
    car_param_map_param["cutDist"] = 15.0
    car_param["mapParam"] = CP.mapParam._make(car_param_map_param.values())

    car_param["minEnableSpeed"] = 0.1  # min_v
    car_param["maxEnableSpeed"] = 8.0  # ref_v
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
