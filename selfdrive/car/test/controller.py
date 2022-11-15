import rospy
from std_msgs.msg import Float32
import pymap3d
import math
from pid import PID


class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=False)

        self.pid = PID()
        # self.purepursuit = PurePursuit()

        self.state = {'x': 0, 'y': 0, 'v': 0, 'yaw': 0}
        self.target = {'v': 0, 'angle': 0}
        self.base = [126.891563662157, 37.5777786160887]

        rospy.Subscriber('/target_v', Float32, self.target_v_cb)
        rospy.Subscriber('/sbg/', Float32, self.target_v_cb)

        self.pub_wheel_angle = rospy.Publisher(
            '/wheel_angle', Float32, queue_size=1)
        self.pub_accel_brake = rospy.Publisher(
            '/accel_brake', Float32, queue_size=1)

    def target_v_cb(self, msg):
        self.target['v'] = msg.data

    def ins_position(self, data):
        x, y, _ = pymap3d.geodetic2enu(data.position.x, data.position.y, 0,
                                       self.base[0], self.base[1], 0)
        self.state.update({'x': x, 'y': y})

    def ins_heading(self, data):
        yaw = math.degrees(data.angle.z)
        self.state.update(
            {'yaw': 90 - yaw if (yaw >= -90 and yaw <= 180) else -270 - yaw})

    def run(self):
        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            # if self.get_odom:
            # if self.final_path is None:
            # wheel_angle = 0.0
            # else:
            # wheel_angle, lah_pt, lah_dist = self.purepursuit.run(self.x, self.y, self.yaw, self.v, self.final_path)
            # lah_viz = LookAheadViz(lah_pt)
            # self.pub_lah.publish(lah_viz)

            # accel_brake = self.pid.run(self.target_v, self.v)
            # self.pub_wheel_angle.publish(Float32(wheel_angle))
            # self.pub_accel_brake.publish(Float32(accel_brake))

            rate.sleep()


if __name__ == "__main__":
    controller = Controller()
    controller.run()
