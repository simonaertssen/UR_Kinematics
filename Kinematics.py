import numpy as np
import time
from numba import jit


@jit(nopython=True)
def T(theta, d, r, alpha):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    cos_a = np.cos(alpha)
    sin_a = np.sin(alpha)
    result = np.zeros((4, 4), np.float64)
    result[0, 0], result[0, 1], result[0, 2], result[0, 3] = cos_t, -sin_t*cos_a,  sin_t*sin_a, r*cos_t
    result[1, 0], result[1, 1], result[1, 2], result[1, 3] = sin_t,  cos_t*cos_a, -cos_t*sin_a, r*sin_t
    result[2, 1], result[2, 2], result[2, 3] = sin_a, cos_a, d
    result[4, 4] = 1
    return result


@jit(nopython=True)
def ForwardKinematics(joint_angles, tool_position=None):
    a, b, c, d, e, f = np.float64(joint_angles)
    if tool_position is not None:
        x, y, z = tool_position
    else:
        x, y, z = np.nan, np.nan, np.nan

    # The joint parameters a, d and alpha can be found here: https://www.universal-robots.com/articles/ur-articles/parameters-for-calculations-of-kinematics-and-dynamics/
    base     = T(theta=a, d=0.089159, r=-0.134,    alpha=np.pi / 2)
    shoulder = T(theta=b, d=0,        r=-0.425,   alpha=0)
    elbow    = T(theta=c, d=-0.119,    r=0,        alpha=0)
    elbowend = T(theta=0, d=0,        r=-0.39225, alpha=0)
    wrist1   = T(theta=d, d=0.09475, r=0,        alpha=np.pi / 2)
    wrist2   = T(theta=e, d=0.09475,  r=0,        alpha=-np.pi / 2)
    wrist3   = T(theta=f, d=0.0815,  r=0,        alpha=0)

    base[0, 3], base[1, 3] = -base[1, 3], base[0, 3]
    shoulder = base.dot(shoulder)
    elbow = shoulder.dot(elbow)
    elbowend = elbow.dot(elbowend)
    wrist1 = elbowend.dot(wrist1)
    wrist2 = wrist1.dot(wrist2)
    wrist3 = wrist2.dot(wrist3)
    X = np.array([0, 0, base[0, 3], shoulder[0, 3], elbow[0, 3], elbowend[0, 3], wrist1[0, 3], wrist2[0, 3], wrist3[0, 3], x])
    Y = np.array([0, 0, base[1, 3], shoulder[1, 3], elbow[1, 3], elbowend[1, 3], wrist1[1, 3], wrist2[1, 3], wrist3[1, 3], y])
    Z = np.array([0, base[2, 3], base[2, 3], shoulder[2, 3], elbow[2, 3], elbowend[2, 3], wrist1[2, 3], wrist2[2, 3], wrist3[2, 3], z])
    return X, Y, Z


def detectCollision(positions):
    X, Y, Z = positions

    # Are we inside of the box?
    if (((-0.832 < X) & (X < 0.490)).any() and ((-0.713 < Y) & (Y < 0.265)).any() and (-0.021 < Z).any()):
        return True
    # Are we bumping into the camera and the light?
    if ((-0.568 < X) & (X < -0.364)).any() and ((-0.266 < Y) & (Y < 0.031)).any() and (0.765 < Z).any():
        return True
        # Are we bumping into the camera and the light?
    if (X < -0.182).any() and (Y < -0.520).any() and (Z < 0.375).any():
        print('Bumping into the screen')
        return True
    return False


def SpeedOfCurrentKinematics():
    start = time.time()
    ForwardKinematics.inspect_types()
    n = 100000
    for _ in range(n):
        toolTip = None
        pos = ForwardKinematics((0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0))
    interval = time.time() - start

    print(n/interval, "iterations per second")


if __name__ == '__main__':
    SpeedOfCurrentKinematics()
    # 17945 iterations per second without 'slots'
    # 17859 iterations per second with 'slots'
    # 17663 iterations per second without for loop
    # 14195 iterations per second by using just a function
    # 54986 iterations per second by using basic Numba, increasing for longer periods
