from typing import NamedTuple

class Positions(NamedTuple):
    latitude : float = 37.45
    longitude : float = 126.6535
    altitude : float = 5.0

class WheelSpeeds(NamedTuple):
    fl: float = 0.0
    fr: float = 0.0
    rl: float =0.0
    rr: float =0.0


class CruiseState(NamedTuple):
    enabled: bool = False
    speed: float = 0.0
    available: bool =  False


class GearShifter(NamedTuple):
    park: int = 1
    drive: int = 2
    neutral: int = 3
    reverse: int = 4


class ButtonType(NamedTuple):
    leftBlinker: int = 1
    rightBlinker: int = 2
    accelCruise: int = 3
    decelCruise: int = 4
    cancel: int = 5
    setCruise: int = 6
    resumeCruise: int = 7


class ButtonEvent(NamedTuple):
    pressed: bool = False
    buttonType: ButtonType = ButtonType()


class State(NamedTuple):
    vEgo: float = 0.0
    aEgo: float = 0.0

    position: Positions = Positions()

    yawRate: float = 0.0
    wheelSpeeds: WheelSpeeds = WheelSpeeds()

    brake: float = 0.0
    brakePressed: bool = False

    steeringAngleDeg: float = 0.0
    steeringTorque: float = 0.0

    cruiseState: CruiseState = CruiseState()

    gearShifter: GearShifter = GearShifter()

    buttonEvent: ButtonEvent = ButtonEvent()

    leftBlinker: bool = False
    rightBlinker: bool = False
