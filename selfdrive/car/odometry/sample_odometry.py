import math

FRONT_TREAD = 1.638 # 1,638mm
REAR_TREAD = 1.647 #  1,647mm
WHEEL_DIAMETER = 0.4826 + 0.10 * 2 # 19 * 0.0254 + tire thick * 2 
ENCODER_RESOLUTION = 46 # 20ms
KMH2MS = 1 / 3.6

pre_enc_cnt_L = 0
pre_enc_cnt_R = 0

pose_x = 0
pose_y = 0
pose_th = 0
# 0~127  overflow 

while True:
    # enc_cnt_L = FRONT LEFT WHEEL ODOMETRY CAN signal 
    # enc_cnt_R = FRONT RIGHT WHEEL ODOMETRY CAN signal 
    
    diff_enc_cnt_L = (enc_cnt_L - pre_enc_cnt_L)
    diff_enc_cnt_R = (enc_cnt_R - pre_enc_cnt_R)
    diff_enc_cnt_L = diff_enc_cnt_L + 128 if diff_enc_cnt_L < 0 else diff_enc_cnt_L
    diff_enc_cnt_R = diff_enc_cnt_R + 128 if diff_enc_cnt_R < 0 else diff_enc_cnt_R

    dL = math.pi * WHEEL_DIAMETER / ENCODER_RESOLUTION * diff_enc_cnt_L
    dR = math.pi * WHEEL_DIAMETER / ENCODER_RESOLUTION * diff_enc_cnt_R
    
    dist = (dR+dL) / 2
    # str_ang = str_ang can signal
    dth = math.atan((dR-dL) / REAR_TREAD) + math.radians(str_ang / 13.73) # here is plus alpha ackerman  //// left + right -

    dx = dist*math.cos(dth)
    dy = dist*math.sin(dth)

    pose_x = dx*math.cos(pose_th) - dy*math.sin(pose_th) + pose_x
    pose_y = dx*math.sin(pose_th) + dy*math.cos(pose_th) + pose_y
    pose_th += dth

    pre_enc_cnt_L = enc_cnt_L
    pre_enc_cnt_R = enc_cnt_R
    
'''
후륜 기준 계산해라.
스티어링도 계산해서 오도메트리 해야된다.
'''



