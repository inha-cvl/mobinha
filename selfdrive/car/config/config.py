import json

class Config(object):

    f = open("fingerprint/niro.json")
    data = json.load(f)
    f.close()

    car_name = data['car_name']
    dbc_path = '../dbc/%s/can.dbc'%(car_name)
    wheel_size = float(data['control_param']['wheel_size'])
