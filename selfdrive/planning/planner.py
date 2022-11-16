from path_planner import PathPlanner
from longitudinal_planner import LongitudinalPlanner
from config.config import Config


def planner():
    config = Config()
    path_planner = PathPlanner(config)
    longitudinal_planner = LongitudinalPlanner(config)

    while True:
        path_planner.update()
        longitudinal_planner.update()


def main():
    planner()


if __name__ == "__main__":
    main()
