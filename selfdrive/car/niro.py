from can_transceiver import CanTransceiver
from car.values import *
import sys


def car(CP):
    can_transceiver = CanTransceiver(CP)

    while True:
        can_transceiver.update()


def main(car):
    try:
        car_class = getattr(sys.modules[__name__], car)
        CP = car_class.CP
    except:
        print("[ERROR] could not laod car values")

    car(CP)


if __name__ == "__main__":
    car = "NIRO"
    main(car)
