from message.messaging import CC

class ControlMaster:
    def __init__(self):
        self.CC = CC
    def update(self):
        car_control = self.CC._asdict()
        self.CC = self.CC._make(car_control.values())