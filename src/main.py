import time
import numpy as np
from lib.Kinematics import ForwardKinematics


def SpeedOfCurrentKinematics():
    start = time.time()
    n = 10000
    for _ in range(n):
        toolTip = None
        pos = ForwardKinematics((0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0))
    interval = time.time() - start
    print(n/interval, "iterations per second")


if __name__ == '__main__':
    SpeedOfCurrentKinematics()
    #  14195 iterations per second by using just a function (benchmark)
    # 480658 iterations per second by using cython
    # 547236 iterations per second by using smaller buffers in matmul
    # 258536
