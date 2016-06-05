import numpy as np

class PathPlanner:

    def __init__(self, map, begin_coord=None, end_coord=None, time_step = 0.010):
        """
        Path planner for generating robot trajectories. Rotate and drive is preferred over
        quintic spline hermapolation for simplicity.
        :param map:
        :param begin_coord:
        :param end_coord:
        :return:
        """
        self.map = map
        self.o_coord = begin_coord
        self.g_coord = end_coord
        self.delta = time_step
        self.find_poses()

    def find_poses(self):
        theta = np.arctan2(*list(reversed(self.g_coord[:2])))
