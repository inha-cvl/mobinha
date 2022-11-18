from typing import NamedTuple


class CarDriving(NamedTuple):
    localPath: list = []
    localLasts: int = len(localPath)-1

    nowLaneID: int = 0
    lidarObjects: list = []

    # State 0 : Ready, 1 : Processing, 2 : Finish
    pathPlanningState: int = 0
    longitudinalPlanningState: int = 0
