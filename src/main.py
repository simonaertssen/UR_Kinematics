import time
import numpy as np
from lib import cythonKinematics


def SpeedOfCurrentKinematics():
    start = time.time()
    n = 100
    for _ in range(n):
        toolTip = None
        time.sleep(0.001)
        pos = cythonKinematics.ForwardKinematics((0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0))
        print(pos)
    interval = time.time() - start
    print(n/interval, "iterations per second")


if __name__ == '__main__':
    SpeedOfCurrentKinematics()
    # 17945 iterations per second without 'slots'
