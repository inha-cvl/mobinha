class Config(object):
    # name = 'KCity'
    name = 'SeoulAutonomousDrivingTestBed'
    # name = 'AlphaCity' 

    # map parameters
    map_path = './map/%s.json'%(name)
    tile_size = 5.0
    cut_dist = 15.0

    # autonomous mode parameters
    eps_en = 1 # handle control(off:0, on:1)
    acc_en = 1 # accel control(off:0, on:1)
    eps_speed = 190

    # velocity plan parameters
    min_v = 7.0 # minimum velocity to drive(km/h)
    ref_v = 15.0 # reference velocity to drive(km/h)

    # PID parameters(longitudinal controller)
    K_P = 0.8
    K_I = 0.003
    K_D = 0.0002

    # PID parameters(longitudinal controller)
    # niro Sangam
    # L = 2.7 # wheel base
    # k1 = 1.5
    # k2 = 0.85 #used in HMG2021
    # Lfc1 = 2.7
    # Lfc2 = 3.0 #used in HMG2021


    # i30
    # L = 2.65
    # k = 1.5
    # k2 = 0.85
    # Lfc = 2.3
    # Lfc2 = 3.0

    # simulation
    # L = 2.65
    # k = 1.2
    # k2 = 0.85
    # Lfc = 2.0
    # Lfc2 = 3.0

    # #2022 Niro test
    # L = 2.72
    # #k1 = 0.85
    # k1 = 0.3
    # k2 = 1.2
    # Lfc1 = 3.0
    # Lfc2 = 4.0

    # 2022 KIAPI niro
    L = 2.72
    #k1 = 0.85
    k1 = 0.237
    #k1 = 3.0
    k2 = 0.237
    Lfc1 = 5.29 #original
    Lfc1 = 8.0
    Lfc2 = 5.29
    k_curva1 = 20.0

    #2022 KIAPI Stage1 Final
    L = 2.72
    k1 = 0.237
    Lfc1 = 8.0
    k_curva1 = 20.0

    #2022 KIAPI Stage2 Final?
    L = 2.72
    k2 = 0.85
    Lfc2 = 5.29
    k_curva2 = 20.0
    
    #for 2charo -- failed
    #Lfc2 = 2.0
    #k2=1.5

    construction_sites = [('424', 0, 20), ('86', 20, 30)]
    # construction_sites = [('90', 10, 30), ('421', 0, 20)]
