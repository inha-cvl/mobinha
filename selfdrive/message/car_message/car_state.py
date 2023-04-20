from typing import NamedTuple


class Positions(NamedTuple):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    latitude: float = 37.45
    longitude: float = 126.6535
    altitude: float = 5.0
    accuracy: float = 99.999


class WheelSpeeds(NamedTuple):
    fl: float = 0.0
    fr: float = 0.0
    rl: float = 0.0
    rr: float = 0.0


class Actuators(NamedTuple):
    steer: float = 0.0
    accel: float = 0.0
    brake: float = 0.0

class ButtonEvent(NamedTuple):
    leftBlinker: int = 0
    rightBlinker: int = 0
    accelCruise: int = 0
    decelCruise: int = 0
    cancel: int = 0
    setCruise: int = 0
    resumeCruise: int = 0


class CarState(NamedTuple):
    vEgo: float = 0.0
    aEgo: float = 0.0
    position: Positions = Positions()

    yawRate: float = 0.0
    pitchRate: float = 0.0
    rollRate: float = 0.0
    wheelSpeeds: WheelSpeeds = WheelSpeeds()

    actuators: Actuators = Actuators()

    cruiseState: int = 0 # 0:Manual,1:Auto

    gearShifter: int = 0  # 0P 1R 2N 3D

    buttonEvent: ButtonEvent = ButtonEvent()
