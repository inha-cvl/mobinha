from typing import NamedTuple


class WheelSpeeds(NamedTuple):
    fl: float
    fr: float
    rl: float
    rr: float


class CruiseState(NamedTuple):
    enabled: bool
    speed: float
    available: bool


class GearShifter(NamedTuple):
    park: 1
    drive: 2
    neutral: 3
    reverse: 4


class ButtonType(NamedTuple):
    leftBlinker: 1
    rightBlinker: 2
    accelCruise: 3
    decelCruise: 4
    cancel: 5
    setCruise: 6
    resumeCruise: 7


class ButtonEvent(NamedTuple):
    pressed: bool
    buttonType: ButtonType


class State(NamedTuple):
    vEgo: float
    aEgo: float

    yawRate: float
    wheelSpeeds: WheelSpeeds

    brake: float
    brakePressed: bool

    steeringAngleDeg: float
    steeringTorque: float

    cruiseState: CruiseState

    gearShifter: GearShifter

    buttonEvent: ButtonEvent

    leftBlinker: bool
    rightBlinker: bool
