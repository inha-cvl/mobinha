from typing import NamedTuple


class MapParam(NamedTuple):
    path: str = ''
    tileSize: float = 0.0
    cutDist: float = 0.0


class LongitudinalTuning(NamedTuple):
    kpBP: float = 0.0
    kpV: float = 0.0
    kiBP: float = 0.0
    kiV: float = 0.0
    kf: float = 0.0
    deadzoneBP: float = 0.0
    deadzoneV: float = 0.0


class LateralParams(NamedTuple):
    torqueBP: float = 0.0
    torqueV: float = 0.0


class LateralPIDTuning(NamedTuple):
    kpBP: float = 0.0
    kpV: float = 0.0
    kiBP: float = 0.0
    kiV: float = 0.0
    kf: float = 0.0


class LateralINDITuning(NamedTuple):
    outerLoopGainBP: float = 0.0
    outerLoopGainV: float = 0.0


class LateralLQRTuning(NamedTuple):
    scale: float = 0.0
    ki: float = 0.0
    dcGain: float = 0.0
    a:float=0.0
    b:float=0.0
    c:float=0.0
    l: float = 0.0
    k: float = 0.0


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
    pid: LateralPIDTuning = LateralPIDTuning()
    indi: LateralINDITuning = LateralINDITuning()
    lqr: LateralLQRTuning = LateralLQRTuning()
    torque: LateralTorqueTuning = LateralTorqueTuning()


class Ecu(NamedTuple):
    eps: int = 0
    abs: int = 1
    fwdRadar: int = 2
    fwdCamera: int = 3
    engine: int = 4


class Param(NamedTuple):
    name: str = ""
    dbc: str = ""
    mapParam: MapParam = MapParam()

    minEnableSpeed: float = 0.0
    maxEnableSpeed: float = 0.0

    mass: float = 0.0
    wheelbase: float = 0.0  # m distance from rear axle to front axle
    # m distance from center of mass to front axle
    centerToFront: float = 0.0
    steerRatio: float = 0.0  # ratio of steering wheel angle to front wheel angle

    longitudinalTuning: LongitudinalTuning = LongitudinalTuning()
    lateralParams: LateralParams = LateralParams()
    lateralTuning: LateralTuning = LateralTuning()

    vEgoStopping: float = 0.0
    vEgoStarting: float = 0.0

    stopAccel: float = 0.0
    startAccel: float = 0.0

    ecu: Ecu = (0, 1, 2, 3, 4)
