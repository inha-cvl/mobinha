class Odom:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.a = 0.0

        self.toggle = True
    def set(self, data):
        self.x, self.y, self.yaw, self.v, self.a = data

    def fake_pose(self):
        if self.toggle:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.v = 0.0
            self.a = 0.0
            self.toggle = False
        else:
            self.x = 0.1
            self.y = 0.1
            self.yaw = 0.1
            self.v = 0.1
            self.a = 0.1
            self.toggle = True
