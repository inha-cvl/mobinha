 #!/usr/bin/env python3
import json
import threading
import numpy as np
import rospy
from local_pkg.msg import Local
from math import hypot, sqrt
from time import sleep
from shared.path import Path

'''
Global path의 곡률을 기반으로 목표 속도를 계산하고, 차량의 위치 index를 계산하는 모듈
부가적으로 맵 취득 위해서도 사용됨 << 팀장에게 문의
'''

class Localizer(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        # 주기 설정
        self.period = 1.0 / rate
        
        # Local 팀으로부터 정보 수신
        rospy.Subscriber('/local_msgs', Local, self.local_callback)

        # 가독성을 위해 주소 복사
        self.shared = parent.shared
        self.ego = parent.shared.ego
        self.global_path = parent.shared.global_path
        self.local_path = parent.shared.local_path
        self.perception = parent.shared.perception

        # map 이름 저장
        self.mapname = parent.args.map
        print("Current map : ", self.mapname)

        # global path를 저장, 곡률 계산, 곡률 기반 map speed를 계산하여 저장(save)
        self.read_global_path(self.mapname, save=True)  # only one time

        # 변수 선언 및 초기화
        self.hAcc = 100000
        self.x = 0
        self.y = 0        
        self.x_list = []
        self.y_list = []

        # 차량의 위치 index는 초기화 할 때부터 정확한 값으로 설정
        self.index_finder()

    # global index와 local index를 계산하여 저장
    def run(self):
        while True:
            self.index_finder()
            self.local_index_finder()
            sleep(self.period)

    # local로부터 정보 수신
    def local_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.hAcc = msg.hAcc
        self.ego.speeed = msg.speeed
        self.ego.heading = msg.heading
        self.ego.orientaion = msg.orientation
        self.ego.dr_x = msg.dr_x
        self.ego.dr_y = msg.dr_y
        self.ego.roll = msg.roll
        self.ego.pitch = msg.pitch
        self.ego.hAcc = msg.hAcc

        self.ego.x = msg.x
        self.ego.y = msg.y
            
        self.ego.gspeed = msg.gspeed

        # map을 취득하고 싶은 경우 on=True로 변경
        self.map_acquire(on=False)
    
    # 맵 정보(index:{x,y,mission}로 이루어진 딕셔너리 형태)를 받아오고 곡률 기반 종속도 계산
    def read_global_path(self, mapname, save=True):
        with open(f"maps/{mapname}.json", 'r') as json_file:
            # 맵 정보 저장
            json_data = json.load(json_file)
            for _, (x, y, mission) in enumerate(json_data.values()):
                self.global_path.x.append(x)
                self.global_path.y.append(y)
                self.global_path.mission.append(mission)
            
            # 곡률 계산, 값이 너무 산발적이라 중간점들만 추출하여 계산
            global_path_tmp = Path()
            for i in range(len(self.global_path.x)):
                if i%2==0:
                    global_path_tmp.x.append(self.global_path.x[i])
                    global_path_tmp.y.append(self.global_path.y[i]) 

            self.data = np.array(self.curvedBaseVelocity(global_path_tmp, 100))
            
            # 중간점들만 추출하여 계산해도 사용하기에는 산발적이라 이동 평균 필터 적용
            filtered_data = self.moving_average(self.data, 15)
            filtered_data2 = self.moving_average(filtered_data, 15)
            filtered_data3 = self.moving_average(filtered_data2, 15)
            self.result = filtered_data3

            # self.ego.map_speed에 계산된 값 저장
            for i in range(len(self.result)-1):
                self.ego.map_speed.append(self.result[i])
                self.ego.map_speed.append((self.result[i]+self.result[i+1])/2)

            # 끝 지점에서 멈추는 용도 및 indexError 방지를 위한 용도
            self.ego.map_speed.append(0)
            self.ego.map_speed.append(0)

            # 이중으로 예외처리
            if len(self.ego.map_speed)<len(self.global_path.x):
                for i in range(abs(len(self.ego.map_speed)-len(self.global_path.x))):
                    self.ego.map_speed.append(0)

            # 계산된 값을 인위적으로 가공하고 싶을 때 offset과 val 수정
            for i, speed in enumerate(self.ego.map_speed):
                # offset을 바꿔가며 확인해보세요
                offset = 4/3
                val = 20-offset*(20-speed)
                if val > 20:
                    val = 20
                elif val < 0:
                    val = 0

                # 전체적으로 값 내림(0~20 -> 0~12)
                val*=0.6

                self.ego.map_speed[i] = int(np.round(val))

        # plot을 위해 값 txt파일로 저장
        if save:
            with open("/home/gigacha/ego_map_speed.txt", 'w') as file:
                file.writelines(str(self.ego.map_speed))
            print("/home/gigacha/ego_map_speed.txt saved !!")
                
    # 차량의 global index 계산
    def index_finder(self):
        min_dis = -1
        min_idx = 0
        save_idx = 0
        
        for i in range(len(self.global_path.x)):
            try:
                dis = hypot(self.global_path.x[i] - self.ego.x, self.global_path.y[i] - self.ego.y)
            except IndexError:
                break
            if (min_dis > dis or min_dis == -1) and save_idx <= i:
                min_dis = dis
                min_idx = i
                save_idx = i

        self.ego.index = min_idx

        try:
            if self.shared.dr_obs_start:
                self.shared.plan.mission = "dr_obs"
            else:
                self.shared.plan.mission_decision = self.global_path.mission[self.ego.index]
        except IndexError:
            print("IndexError from index_finder!!")

    # 차량의 local index 계산
    def local_index_finder(self):
        if len(self.local_path.x) != 0:
            min_dis = -1
            min_idx = 0
            save_idx = 0
            for i in range(len(self.local_path.x)): 
                try:
                    dis = hypot(
                        self.local_path.x[i] - self.ego.x, self.local_path.y[i] - self.ego.y)
                except IndexError:
                    break
                if (min_dis > dis or min_dis == -1) and save_idx <= i:
                    min_dis = dis
                    min_idx = i
                    save_idx = i
            self.ego.local_index = min_idx
        else:
            self.ego.local_index = 0  

    # 곡률 계산, point_num은 곡률을 계산할 점 앞뒤로 몇 개를 고려할 것인지 지정하는 변수
    def curvedBaseVelocity(self, gloabl_path, point_num, plot=False):
        car_max_speed = 20
        out_vel_plan = []
        r_list = []
        tmp = []

        for i in range(0,point_num):
            out_vel_plan.append(car_max_speed)
            r_list.append(0)

        for i in range(point_num, len(gloabl_path.x) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.x[i+box]
                y = gloabl_path.y[i+box]
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #TODO: (6) 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T
            try:
                a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
                a = a_matrix[0]
                b = a_matrix[1]
                c = a_matrix[2]
                r = sqrt(abs(a*a+b*b-c))
            except np.linalg.LinAlgError:
                r = 0.01

            tmp.append(r)

        # 곡률 계산한 것에 필터 적용한 곳
        result = self.moving_average(np.array(tmp), 15)

        for r in result:
            v_max = sqrt(r*9.8)
            if v_max > car_max_speed:
                v_max = car_max_speed
            
            out_vel_plan.append(v_max)

        for i in range(len(self.global_path.x) - point_num, len(self.global_path.x)-10):
            out_vel_plan.append(20)

        for i in range(len(self.global_path.x) - 10, len(self.global_path.x)):
            out_vel_plan.append(0)

        return out_vel_plan

    # 이동 평균 필터
    def moving_average(self, data, window_size):
        # 이동 평균 계산을 위해 필요한 윈도우 크기보다 작은 경우 예외 처리
        if len(data) < window_size:
            raise ValueError("데이터 크기가 윈도우 크기보다 작습니다.")
        
        # 이동 평균 계산
        weights = np.repeat(1.0, window_size) / window_size
        ma = np.convolve(data, weights, 'valid')
        
        result = []
        for i in range(window_size//2):
            result.append(data[i])
        for val in ma:
            result.append(val)
        for i in range(window_size//2):
            result.append(data[i])

        return result
    
    # 맵 취득하는 함수, 원하는 만큼 움직인 뒤 터미널에 출력되는 x=[...], y=[...]을 복사하여 mapmaker에 넣고 실행하면 된다.
    def map_acquire(self, on=False):
        if on:
            if len(self.x_list)==0:
                self.x_list.append(self.ego.x)
                self.y_list.append(self.ego.y)
                print("x=", self.x_list)
                print("y=", self.y_list)

            distance = ((self.ego.x-self.x_list[-1])**2 + (self.ego.y-self.y_list[-1])**2)**0.5
            if distance>0.5:
                self.x_list.append(round(self.ego.x,2))
                self.y_list.append(round(self.ego.y,2))
                print("x=", self.x_list)
                print("y=", self.y_list)
                print("")


