import time
import numpy as np

import os, sys
sys.path.append(os.getcwd())  # Append the current path tp PYTHONPATH to include parent directories
from KinematicsLib.KinematicsModule import ForwardKinematics, detectCollision


def SpeedOfCurrentKinematics():
    start = time.time()
    n = 1000000
    for _ in range(n):
        toolTip = None
        pos = ForwardKinematics((0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0))
    interval = time.time() - start
    print(n/interval, "iterations per second")


def SpeedOfCollisionDetection():
    start = time.time()
    X = [0.1]*9
    Y = [0.1]*9
    Z = [0.1]*9

    n = 100000
    for _ in range(n):
        toolTip = None
        collision = detectCollision((X, Y, Z))
    interval = time.time() - start
    print(n/interval, "iterations per second")


if __name__ == '__main__':
    # SpeedOfCurrentKinematics()
    #  14195 iterations per second by using just a function (benchmark)
    # 400227 iterations per second by using cython
    # 499907 iterations per second by using cython with c functions
    # 547236 iterations per second by using smaller buffers in matmul

    SpeedOfCollisionDetection()
    # 723004 iterations per second for the simple Cython implementation
