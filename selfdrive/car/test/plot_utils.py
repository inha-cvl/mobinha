import rospy

def state_controller(self):
    while not rospy.is_shutdown():
        cmd = input('99: PA |88: LON |77: ALL\n \
                    1001: reset\n1000: over\n')
        cmd = int(cmd)
        if cmd == 99: 
            self.reset_trigger()
            self.PA_enable = 1
            self.LON_enable = 0
            self.brake = 0
            self.accel = 0
            self.reset = 0
        elif cmd == 88:
            self.reset_trigger()
            self.PA_enable = 0
            self.LON_enable = 1
            self.brake = 0
            self.accel = 0
            self.reset = 0
        elif cmd == 77: 
            self.reset_trigger()
            self.PA_enable = 1
            self.LON_enable = 1
            self.brake = 0
            self.accel = 0
            self.reset = 0
        elif cmd == 1001:
            self.reset_trigger()
        elif cmd == 1000:
            exit(0)