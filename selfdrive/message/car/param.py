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
    kpBP: list = [0.0]
    kpV: list = [0.0]
    kiBP: list = [0.0]
    kiV: list = [0.0]
    kf: float = 0.0


class LateralINDITuning(NamedTuple):
    outerLoopGainBP: list = [0.0]
    outerLoopGainV: list = [0.0]


class LateralLQRTuning(NamedTuple):
    scale: float = 0.0
    ki: float = 0.0
    dcGain: float = 0.0
    l: list = [0.0]
    k: list = [0.0]


class LateralTorqueTuning(NamedTuple):
    useSteeringAngle: bool = False
    kp: float = 0.0
    ki: float = 0.0
    friction: float = 0.0
    kf: float = 0.0
    steeringAngleDeadzoneDeg: float = 0.0
    latAccelFactor: float = 0.0
    latAccelOffset: float = 0.0


class LateralTuning(NamedTuple):
    pid: LateralPIDTuning = ([0.0], [0.0], [0.0], [0.0], 0.0)
    indi: LateralINDITuning = ([0.0], [0.0])
    lqr: LateralLQRTuning = (0.0, 0.0, 0.0, [0.0], [0.0])
    torque: LateralTorqueTuning = (False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class Ecu(NamedTuple):
    eps: int = 0
    abs: int = 1
    fwdRadar: int = 2
    fwdCamera: int = 3
    engine: int = 4


class Param(NamedTuple):
    name: str = ''
    fingerprint: str = ''

    wheelbase: float = 0.0  # [m] distance from rear axle to front axle
    # [m] distance from center of mass to front axle
    centerToFront: float = 0.0
    steerRatio: float = 0.0  # [] ratio of steering wheel angle to front wheel angle

    longitudinalTuning: LongitudinalTuning = (
        [0.0], [0.0], [0.0], [0.0], 0.0, 0.0, 0.0)
    lateralParams: LateralParams = ([0.0], [0.0])
    lateralTuning: LateralTuning = (([0.0], [0.0], [0.0], [0.0], 0.0), ([0.0], [
                                    0.0]), (0.0, 0.0, 0.0, [0.0], [0.0]), (False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    vEgoStopping: float = 0.0
    vEgoStarting: float = 0.0

    stopAccel: float = 0.0
    startAccel: float = 0.0

    ecu: Ecu = (0, 1, 2, 3, 4)
