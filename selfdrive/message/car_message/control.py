from typing import NamedTuple


class LongControlState(NamedTuple):
    off: int = 0
    pid: int = 1
    stopping: int = 2
    starting: int = 3


class Actuators(NamedTuple):
    brake: float = 0.0
    steer: float = 0.0
    speed: float = 0.0
    accel: float = 0.0
    longControlState: LongControlState = (0, 1, 2, 3)


class CruiseControl(NamedTuple):
    cancel: bool = False
    resume: bool = False
    override: bool = False


class Control(NamedTuple):
    enabled: bool = False
    latActive: bool = False
    longActive: bool = False
    actuators: Actuators = (0.0, 0.0, 0.0, 0.0, (0, 1, 2, 3))
    cruiseControl: CruiseControl = (False, False, False)