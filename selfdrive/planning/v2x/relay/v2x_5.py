#!/usr/bin/python
import json 
import struct
import socket
import time
import datetime
import signal
import rospy
import os
import sys
from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgGpsHdt
from shapely.geometry import Point, Polygon
from std_msgs.msg import Int16MultiArray

HOST = 'localhost'
PORT = 9100

class V2X:
    def __init__(self):
        self.pvd_temp = '{"startVector":{"long":1268905411,"lat":375781661,"heading":8800, "speed":{"transmisson":"unavailable","speed":738}},"vehicleType":{"vehicleType":"cars"},"snapshots":[{"thePosition":{"long":1268905411,"lat":375781661, "heading" : 8800,"speed":{"transmisson":"unavailable","speed":738}},"dataSet":{"lights":{"value":"C800","length":9},"wipers":{"statusFront":"high","rateFront":100,"statusRear":"low","rateRear":50},"brakeStatus":{"wheelBrakes":"C0","traction":"on","abs":"on","scs":"on","brakeBoost":"off","auxBrakes":"off"},"brakePressure":"minPressure"}}]}'
        self.pvd = json.loads(self.pvd_temp)
        self.toc = 0
        self.sock = None
        self.bag = {'long_bag' : 0, 'lat_bag' : 0}
        self.head = 0
        # self.msg = False
        region_lat = (37.57709155,37.57680657,37.57613146,37.57590435,37.57601075,37.57580667,37.57518631,37.57489760,37.57520498,37.57491502,37.57604373,37.57574755,37.57632498,37.57607484,37.57711644,37.57690240,37.57726826,37.57707786,37.57814435,37.57786435,37.57818044,37.57852515,37.57953190,37.57990772,37.58016905,37.57994630,37.58130399,37.58109119,37.58153172,37.58132390,37.58214709,37.58193927,37.58195421,37.58223918,37.58148816,37.58178683,37.58176070,37.58146079,37.58083732,37.58054488,37.58035448,37.58065563,37.57940123,37.57968746,37.57937759,37.57915484,37.57857990,37.57836835,37.57853510,37.57830301,37.57812444,37.57791288,37.57809208,37.57786808,37.57747484,37.57724586,37.57748977,37.57713386,37.57829928,37.57791848)
        region_long = (126.8919563,126.8923321,126.8910740,126.8913490,126.8909639,126.8912451,126.8900461,126.8904306,126.8900187,126.8897400,126.8886959,126.8884184,126.8883474,126.8886984,126.8892316,126.8895415,126.8893735,126.8896958,126.8901886,126.8905432,126.8901948,126.8905121,126.8878801,126.8882560,126.8879424,126.8882609,126.8891072,126.8893672,126.8893312,126.8896136,126.8899216,126.8902875,126.8903161,126.8905078,126.8913260,126.8915176,126.8915786,126.8913932,126.8929687,126.8926837,126.8928853,126.8932636,126.8943711,126.8946511,126.8943550,126.8946399,126.8935399,126.8938983,126.8934994,126.8938454,126.8930831,126.8934142,126.8930558,126.8933818,126.8923738,126.8927372,126.8923415,126.8919644,126.8909776,126.8906254)
        region = zip(region_lat,region_long)
        self.region_poly =(0,Polygon(region[0:4]),Polygon(region[4:8]),Polygon(region[8:12]),Polygon(region[12:16]),
        Polygon(region[16:20]),Polygon(region[20:24]),Polygon(region[24:28]),Polygon(region[28:32]),
        Polygon(region[32:36]),Polygon(region[36:40]),Polygon(region[40:44]),Polygon(region[44:48]),
        Polygon(region[48:52]),Polygon(region[52:56]),Polygon(region[56:60]))

        rospy.init_node('V2X_node', anonymous = True)
        rospy.Subscriber("sbg/ekf_nav",SbgEkfNav, self.ekf_nav)
        rospy.Subscriber("sbg/gps_hdt",SbgGpsHdt, self.gps_hdt)
        self.SPaT = rospy.Publisher('/SPaT_msg', Int16MultiArray, queue_size = 1)
        
    def spat(self, data):
        gps_now = Point(self.bag['lat_bag'],self.bag['long_bag'])
        spat_msg = Int16MultiArray()  ## convert 'eventstate' to int
        os.system('clear')
        print('SPaT-----------------------------------------------------')
        for sections in data['intersections']:
            print(sections['name'])
            for states in sections['states']:
                state = states['state-time-speed']
                eventstate = state[0]['eventState']
                es_int = self.signalstate(eventstate)
                timing = state[0]['timing']['minEndTime']
                if states['signalGroup'] == 70 and (gps_now.within(self.region_poly[4]) or gps_now.within(self.region_poly[5]) or gps_now.within(self.region_poly[7]) or gps_now.within(self.region_poly[8])):
                    print"signal 70 :",eventstate
                    print"minEndTIme :",timing
                    print(es_int,timing)
                    print(states['movementName'])
                    spat_msg.data.extend([es_int,timing])

                elif states['signalGroup'] == 80 and (gps_now.within(self.region_poly[9]) or gps_now.within(self.region_poly[10]) or gps_now.within(self.region_poly[11])):                   
                    print"signal 80 :",eventstate
                    print"minEndTIme :",timing
                    print(es_int,timing)
                    print(states['movementName'])
                    spat_msg.data.extend([es_int,timing])    

                elif states['signalGroup'] == 50 and (gps_now.within(self.region_poly[1]) or gps_now.within(self.region_poly[2]) or gps_now.within(self.region_poly[12]) or gps_now.within(self.region_poly[13]) or gps_now.within(self.region_poly[14])):
                    print"signal 50 :",eventstate
                    print"minEndTIme :",timing
                    print(es_int,timing)
                    print(states['movementName'])
                    spat_msg.data.extend([es_int,timing])    

                elif states['signalGroup'] == 60 and (gps_now.within(self.region_poly[3]) or gps_now.within(self.region_poly[6]) or gps_now.within(self.region_poly[15])):
                    print"signal 60 :",eventstate
                    print"minEndTIme :",timing
                    print(es_int,timing)
                    print(states['movementName'])
                    spat_msg.data.extend([es_int,timing])    

        self.SPaT.publish(spat_msg)
        spat_msg.data = []
    

    def signalstate(self, signal):
        temp = 0
        if signal == 'stop-Then-Proceed':
            temp = 1
        elif signal == 'stop-And-Remain':
            temp = 2
        elif signal == 'pre-Movement':
            temp = 3
        elif signal == 'permissive-Movement-Allowed':
            temp = 4
        elif signal == 'protected-Movement-Allowed':
            temp = 5
        elif signal == 'permissive-clearance':
            temp = 6
        elif signal == 'protected-clearance':
            temp = 7
        else:
            pass
        return temp

    def ekf_nav(self,data):
        self.bag['long_bag'] = data.position.y
        self.bag['lat_bag'] = data.position.x
            
    def gps_hdt(self,data):
        head = data.true_heading
        if (head >= 0 and head <= 180):
            self.head = head + 180
        else:
            self.head = head - 180
            # self.msg = True

    def getMsg(self): 
        msg = [0x61]
        data = self.sock.recv(9192)
        leng, typ= struct.unpack('>IB', data[:5]) 
        # print(hex(typ))
        if typ in msg:
            real_data = data[5:]
            real_data = json.loads(real_data.decode())
            self.spat(real_data)

    def dumper(self):
        self.pvd['startVector']['long'] = int(round(self.bag['long_bag'] * 10000000))
        self.pvd['startVector']['lat'] = int(round(self.bag['lat_bag'] * 10000000))
        self.pvd['startVector']['heading'] = int(round(self.head / 0.0125 ))
        self.pvd['snapshots'][0]['thePosition']['long'] = int(round(self.bag['long_bag'] * 10000000))
        self.pvd['snapshots'][0]['thePosition']['lat'] = int(round(self.bag['lat_bag'] * 10000000))
        self.pvd['snapshots'][0]['thePosition']['heading'] = int(round(self.head / 0.0125 ))
        data = json.dumps(self.pvd)
        return data

    def packer(self, data):
        packet = struct.pack('>IB{}s'.format(len(data)), len(data)+5, 0x01, data.encode())
        return packet

    def timer(self, tic):
        if (time.time() - self.toc > tic):
            self.toc = time.time()
            return True
        else:
            return False

    def createConn(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        print('connected')

    def daemon(self):
        if (self.timer(0.1) == True):
            data = self.dumper()
            data = self.packer(data)
            self.sock.sendall(data)
            # self.msg = False

def signal_handler(sig, frame):
    print('\nPressed CTRL + C !')
    sys.exit(0)

if __name__ == '__main__':
    V2X = V2X()
    V2X.createConn()
    while 1:
        try:
            signal.signal(signal.SIGINT, signal_handler)
            V2X.daemon()        
            V2X.getMsg()
        except ValueError:
            pass
        except Exception as e:
            print(e)
            print(type(e))
            exit()

