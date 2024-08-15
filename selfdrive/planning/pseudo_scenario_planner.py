import rospy
from std_msgs.msg import Float32
import numpy as np

class PseudoPlanner:
    def __init__(self):
        ## Subscribers
        rospy.Subscriber("/Ego_topic", Float32, self.ego_pos_cb)
        self.ego_pos = None

        rospy.Subscriber("/roundAbout_stopline_pos", Float32, self.roundAbout_stopline_pos_cb) # not used
        self.roundAbout_stopline_pos = [35.87,1827.29]

        rospy.Subscriber("/trafficLight_stopline_pos", Float32, self.trafficLight_stopline_pos_cb) # not used
        self.trafficLight_stopline_pos = [58.75, 1,180.78]

        rospy.Subscriber("/GetTrafficLightStatus", , self.trafficLight_type_cb)
        self.trafficLight_type = None
        
        rospy.Subscriber("/Object_topic", Float32, self.obstacle_pos_cb)
        self.obstacle_pos = None
        self.s2obstacle = 100

        ## Scenario
        self.scenario = None

        ## Publishers
        self.target_s_pub = rospy.Publisher("/target_s", Float32, queue_size=1)
        self.target_s = 100

        self.target_v_pub = rospy.Publisher("/target_v", Float32, queue_size=1)
        self.target_v = 0

        ## Local variables
        self.roundabout_stopped = False


    def ego_pos_cb(self, msg):
        self.ego_pos = msg
        self.ego_vel = msg


    def roundAbout_stopline_pos_cb(self, msg):
        self.roundabout_stopline_pos = msg
        self.s2roundAbout = np.sqrt((self.roundAbout_stopline_pos[0]-self.ego_pos[0])**2 + \
                                    (self.roundAbout_stopline_pos[1]-self.ego_pos[1])**2)


    def trafficLight_stopline_pos_cb(self, msg):
        self.trafficLight_stopline_pos = msg
        self.s2trafficLight = np.sqrt((self.trafficLight_stopline_pos[0]-self.ego_pos[0])**2 + \
                                      (self.trafficLight_stopline_pos[1]-self.ego_pos[1])**2)


    def trafficLight_type_cb(self, msg):
        self.trafficLight_type = msg
        

    def obstacle_pos_cb(self, msg):
        self.obstacle_pos = msg
        if self.obstacle_pos on the link: # 해줘!!
            self.s2obstacle = np.sqrt((self.obstacle_pos[0]-self.ego_pos[0])**2 + \
                                      (self.obstacle_pos[1]-self.ego_pos[1])**2)
        else:
            self.s2obstacle = 100
        

    def scenario_selector(self):
        self.scenario = "roundAbout"
        # self.scenario = "trafficLight"
        # self.scenario = "obstacle"


    def roundAbout_target_s_planner(self):
        check stopped code 부터 하믄댐
        if not self.roundabout_stopped: # only for morai
            self.target_s = min(self.s2obstacle, self.s2roundAbout)
        else:
            self.target_s = self.s2obstacle 


    def trafficLight_target_s_planner(self):
        if self.trafficLight_type not in [48, 20, 16]: #직좌, 직황, 직
            self.target_s = min(self.s2obstacle, self.s2trafficLight)
        else:
            self.target_s = self.s2obstacle
    

    def obstacle_target_s_planner(self):
        self.target_s = self.s2obstacle
    

    def target_s_planner(self):
        if self.scenario == "roundAbout":
            self.roundAbout_target_s_planner()
        elif self.scenario == "trafficLight":
            self.trafficLight_target_s_planner()
        elif self.scenario == "obstacle":
            self.obstacle_target_s_planner()
        else:
            print("Error: no scenario matchjed at function 'target_s_planner'")

    
    def target_v_planner(self):
        self.target_v = 10 # 실차에는 곡률기반 + 최고최저속도 반영
    

    def tmp_v_planner_for_morai(self):
        if self.target_s < 3:
            self.target_v = 0
    

    def target_values_publish(self):
        self.target_s_pub.publish(self.target_s)
        self.target_v_pub.publish(self.target_v)

    
    def run(self):
        self.scenario_selector()
        self.target_s_planner()
        self.target_v_planner()
        self.tmp_v_planner_for_morai()
        self.target_values_publish()



if __name__ == "__main__":
    rospy.init_node("longitudinal_planner_for_MORAI")
    pseudoPlanner = PseudoPlanner()
    while not rospy.is_shutdown():
        pseudoPlanner.run()