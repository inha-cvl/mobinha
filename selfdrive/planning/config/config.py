class Config(object):
    name = 'songdo'
    # name = 'SeoulAutonomousDrivingTestBed'
    # name = 'AlphaCity' 

    # map parameters
    map_path = '../map/%s.json'%(name)
    tile_size = 5.0
    cut_dist = 15.0

    # autonomous mode parameters
    eps_en = 1 # handle control(off:0, on:1)
    acc_en = 1 # accel control(off:0, on:1)
    eps_speed = 190

    # velocity plan parameters
    min_v = 41.0 # minimum velocity to drive(km/h)
    ref_v = 48.0 # reference velocity to drive(km/h)

    # PID parameters(longitudinal controller)
    K_P = 0.75
    K_I = 0.15
    K_D = 0.06

    # PID parameters(longitudinal controller)
    # niro
    L = 2.72 # wheel base
    k = 1.5
    k2 = 0.85
    Lfc = 1.5
    Lfc2 = 3.0

    # i30
    # L = 2.65
    # k = 1.5
    # k2 = 0.85
    # Lfc = 2.3
    # Lfc2 = 3.0

    # simulation
    #L = 2.65
    #k = 1.2
    #k2 = 0.85
    #Lfc = 2.0
    #Lfc2 = 3.0

    construction_sites = [('424', 0, 20), ('86', 20, 30)]
    # construction_sites = [('90', 10, 30), ('421', 0, 20)]
