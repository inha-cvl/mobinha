from cereal.messaging import SubMaster
import numpy as np
import os
import rospy
import signal
import sys
import time
import utm
import config
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from std_msgs.msg import Bool, Int8, Int16MultiArray, Int16
from sbg_driver.msg import SbgEkfNav

os.environ["ZMQ"] = "1"
ZONE = config.kcity # TOR zone

class LaneCheck:
    def __init__(self):
        self.sm = None
        self.temp = {'0': 0, '1': 0, '2': 0, '3': 0}

        rospy.init_node('laneline', anonymous=True)

        self.lane_state_pub = rospy.Publisher('/unstable_lane', Bool, queue_size=1)
        self.lane_warn_pub = rospy.Publisher('/lane_warn', Int8, queue_size=1)
        self.lkas_state = rospy.Publisher('/lkas', Bool, queue_size=1)
        self.op_fcw = rospy.Publisher('/op_fcw', Bool, queue_size = 1)

        rospy.Subscriber('/can_record', Int16MultiArray, self.can_record_callback)
        rospy.Subscriber('/sbg/ekf_nav', SbgEkfNav, self.ekf_nav_callback)
       
        self.curvature_pub = rospy.Publisher('/curvature', Int16, queue_size=1)
        self.curvature = 0

        self.onLane = False
        self.warnLane = 0
        self.lkas = Bool()
        self.lkas.data = False
        self.vel = 0
        self.lane_prob = False
        self.edge = 0
        self.joining = False 

        self.area = Polygon(ZONE)
    
    def can_record_callback(self, msg):
        self.vel = msg.data[5]

    def ekf_nav_callback(self, msg):
        x,y,_,_ = utm.from_latlon(msg.latitude, msg.longitude, 52, 'N')
        car = Point(y,x)
        self.joining = self.area.contains(car)

    def reconnect(self):
        addr = '192.168.101.100'
        self.sm = SubMaster(['carState', 'longitudinalPlan', 'carControl', 'radarState', 'liveCalibration', 'controlsState', 'carParams',
            'liveTracks', 'modelV2', 'liveParameters', 'lateralPlan', 'sendcan', 'gpsLocationExternal',
            'clocks', 'thumbnail', 'roadCameraState', 'driverState', 'procLog', 'ubloxGnss', 'ubloxRaw',
            'cameraOdometry', 'carEvents', 'driverCameraState', 'driverMonitoringState'],
            addr=addr)

    def get_lane_lines(self):
        self.sm.update(0)
        if(time.time() - self.sm.rcv_time['modelV2'] > 1):
            self.lkas.data = True
            self.lkas_state.publish(self.lkas)
            self.reconnect()
        else:
            self.lkas.data = False
            self.lkas_state.publish(self.lkas)
            if self.sm['modelV2']:               
                if self.sm['modelV2'].laneLines:
                    for i, _ in enumerate(self.sm['modelV2'].laneLines):
                        self.temp[str(i)] = _
                    x = self.temp['1'].x
                    line1s = self.temp['1'].y
                    line2s = self.temp['2'].y

                    # 2. Calculate Whether a Car is on the Lane or Not
                    self.onLane = False
                    #Delete On Lane Checker with Lane Width 
                    # lane_width = line2s[0]-line1s[0] 
                    # if (lane_width > 3.7):
                    #     self.onLane = True
                    if self.lane_prob:
                        self.onLane = True
                    if (line1s[0]>-0.7):
                        self.onLane = True
                    if (line2s[0]<0.7):
                        self.onLane = True
                    if ((self.curvature < 90) and self.vel < 25):
                        self.onLane = True

                    # 3. Calculate Lane Departure
                    if self.sm['carState'].leftBlinker or self.sm['carState'].rightBlinker:
                        self.warnLane = 0
                    else:    
                        if (-0.9<line1s[0]<-0.75 or 0.75<line2s[0]<0.9):
                            self.warnLane = 1
                        elif (-0.75<line1s[0] or 0.75>line2s[0]) or self.lane_prob or (self.curvature < 60) or self.joining:
                            self.warnLane = 2
                        else:
                            self.warnLane = 0
                    
                     # 4. Calculate Curvature 
                    xx = np.array(x)[3:16] # for calculate forward lane (5~15)
                    lefty = np.array(line1s)[3:16]
                    righty = np.array(line2s)[3:16]
                    left_fit_cr = np.polyfit(xx, lefty, 2) # return poltnomial coefficient
                    right_fit_cr = np.polyfit(xx, righty, 2)
                    left_curvated = min(int(((1+(2*left_fit_cr[0]+left_fit_cr[1])**2)**1.5)/np.absolute(2*left_fit_cr[0])), 30000) # calculate curvature
                    right_curvated = min(int(((1+(2*right_fit_cr[0]+right_fit_cr[1])**2)**1.5)/np.absolute(2*right_fit_cr[0])), 30000)
                    self.curvature = min(left_curvated, right_curvated)


            self.curvature_pub.publish(self.curvature)
            
            self.lane_state_pub.publish(self.onLane)
            self.lane_warn_pub.publish(self.warnLane)                   

    def get_fcw_events(self):
        FCW1 = False
        FCW2 = False
        FCW = False
        if self.sm['longitudinalPlan'] :
            FCW1 = self.sm['longitudinalPlan'].fcw
        if self.sm['modelV2']:
            FCW2 = self.sm['modelV2'].meta.hardBrakePredicted      
        if FCW1 or FCW2 :
            FCW = True
        self.op_fcw.publish(FCW)
    
    def get_lane_prob(self):
        if self.sm['modelV2'].laneLineProbs:
            prob = (self.sm['modelV2'].laneLineProbs[1]+self.sm['modelV2'].laneLineProbs[2])/2
            # When there are no lanes 
            self.lane_prob = True if prob < 0.15 else False

    def get_edge(self):
        if self.sm['modelV2'].roadEdges:
            for i, _ in enumerate(self.sm['modelV2'].roadEdges):
                self.temp[str(i)] = _
            line2s = self.temp['1'].y
            self.edge = line2s[1]

def signal_handler(sig, frame):
    sys.exit(0)

if __name__ == "__main__":
    lc = LaneCheck()
    lc.reconnect()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:  
            signal.signal(signal.SIGINT, signal_handler)
            lc.get_lane_lines()
            lc.get_fcw_events()
            lc.get_lane_prob()
            lc.get_edge()
            rate.sleep()
        except Exception as e:
            exit()
