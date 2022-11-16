
class LongitudinalPlanner:
    def __init__(self, CP):
        self.factor = CP.wheel_size

    def run(self):
        print(self.config.map_path)
