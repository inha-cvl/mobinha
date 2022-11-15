from typing import NamedTuple


class LongitudinalTuning(NamedTuple):
    kpBP: list
    kpV: list
    kiBP: list
    kiV: list
    kf: float
    deadzoneBP: float
    deadzoneV: float


class LateralParams(NamedTuple):
    torqueBP: list
    torqueV: list


class LateralPIDTuning(NamedTuple):
    kpBP: list
    kpV: list
    kiBP: list
    kiV: list
    kf: float


class LateralINDITuning(NamedTuple):
    outerLoopGainBP: list
    outerLoopGainV: list


class LateralLQRTuning(NamedTuple):
    scale: float
    ki: float
    dcGain: float
    l: list
    k: list


class LateralTorqueTuning(NamedTuple):
    useSteeringAngle: bool
    kp: float
    ki: float
    friction: float
    kf: float
    steeringAngleDeadzoneDeg: float
    latAccelFactor: float
    latAccelOffset: float


class LateralTuning(NamedTuple):
    pid: LateralPIDTuning
    indi: LateralINDITuning
    lqr: LateralLQRTuning
    torque: LateralTorqueTuning


class Ecu(NamedTuple):
    eps: 0
    abs: 1
    fwdRadar: 2
    fwdCamera: 3
    engine: 4


class Param(NamedTuple):
    name: str
    fingerprint: str

    wheelbase: float  # [m] distance from rear axle to front axle
    centerToFront: float  # [m] distance from center of mass to front axle
    steerRatio: float  # [] ratio of steering wheel angle to front wheel angle

    longitudinalTuning: LongitudinalTuning
    lateralParams: LateralParams
    lateralTuning: LateralTuning

    vEgoStopping: float
    vEgoStarting: float

    stopAccel: float
    startAccel: float

    ecu: Ecu
