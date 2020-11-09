import time
import numpy as np
from lib.Kinematics import ForwardKinematics


def SpeedOfCurrentKinematics():
    start = time.time()
    n = 100000
    for _ in range(n):
        toolTip = None
        pos = ForwardKinematics((0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0))
        print(pos)
    interval = time.time() - start
    print(n/interval, "iterations per second")


if __name__ == '__main__':
    SpeedOfCurrentKinematics()
    # 17945 iterations per second without 'slots'
