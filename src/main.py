import time
from Kinematics import ForwardKinematics


def SpeedOfCurrentKinematics():
    start = time.time()
    n = 100
    for _ in range(n):
        toolTip = None
        time.sleep(0.001)
        # pos = ForwardKinematics((0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0))
    interval = time.time() - start
    print(n/interval, "iterations per second")


if __name__ == '__main__':
    SpeedOfCurrentKinematics()
    # 17945 iterations per second without 'slots'
