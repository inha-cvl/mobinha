
if __package__ is None:
    import sys
    from os import path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    from message.messaging import CP


class NIRO:
    dbc = 'dbc/niro/can.dbc'
    car_param = CP
    car_param.name = 'niro'
    car_param.wheelbase = 2.72  # L
    car_param.centerToFront
    car_param.steerRatio = 13.73
