import numpy as np
import time


def T(theta, d, r, alpha):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    cos_a = np.cos(alpha)
    sin_a = np.sin(alpha)
    return np.array([[cos_t, -sin_t*cos_a,  sin_t*sin_a, r*cos_t],
                     [sin_t,  cos_t*cos_a, -cos_t*sin_a, r*sin_t],
                     [    0,        sin_a,        cos_a,       d],
                     [    0,           0,           0,         1]])


def ForwardKinematics(joint_angles):
    a, b, c, d, e, f = joint_angles
    # print([np.round(angle * 180/3.14) for angle in joint_angles])
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
    X = np.array([0, 0, base[0, 3], shoulder[0, 3], elbow[0, 3], elbowend[0, 3], wrist1[0, 3], wrist2[0, 3], wrist3[0, 3]])
    Y = np.array([0, 0, base[1, 3], shoulder[1, 3], elbow[1, 3], elbowend[1, 3], wrist1[1, 3], wrist2[1, 3], wrist3[1, 3]])
    Z = np.array([0, base[2, 3], base[2, 3], shoulder[2, 3], elbow[2, 3], elbowend[2, 3], wrist1[2, 3], wrist2[2, 3], wrist3[2, 3]])

    # base     = T(theta=a, d=0.089159, r=0,        alpha=np.pi / 2)
    # shoulder = T(theta=b, d=0,        r=-0.425,   alpha=0)
    # elbow    = T(theta=c, d=0,        r=-0.39225, alpha=0)
    # wrist1   = T(theta=d, d=0.10915,  r=0,        alpha=np.pi / 2)
    # wrist2   = T(theta=e, d=0.09465,  r=0,        alpha=-np.pi / 2)
    # wrist3   = T(theta=f, d=0.0823,   r=0,        alpha=0)
    # shoulder = base.dot(shoulder)
    # elbow = shoulder.dot(elbow)
    # wrist1 = elbow.dot(wrist1)
    # wrist2 = wrist1.dot(wrist2)
    # wrist3 = wrist2.dot(wrist3)
    # X = np.array([0, 0, base[0, 3], shoulder[0, 3], elbow[0, 3], wrist1[0, 3], wrist2[0, 3], wrist3[0, 3]])
    # Y = np.array([0, 0, base[1, 3], shoulder[1, 3], elbow[1, 3], wrist1[1, 3], wrist2[1, 3], wrist3[1, 3]])
    # Z = np.array([0, base[2, 3], base[2, 3], shoulder[2, 3], elbow[2, 3], wrist1[2, 3], wrist2[2, 3], wrist3[2, 3]])
    # print(X[-1], Y[-1], Z[-1])
    return X, Y, Z


def SpeedOfCurrentKinematics():
    start = time.time()
    for _ in range(100000):
        pos = ForwardKinematics([0, -np.pi/2, 0, -np.pi/2, 0, 0])
    interval = time.time() - start
    print(100000/interval, "iterations per second")


if __name__ == '__main__':
    SpeedOfCurrentKinematics()
    # 17945 iterations per second without 'slots'
    # 17859 iterations per second with 'slots'
    # 17663 iterations per second without for loop
    # 14195 iterations per second by using just a function
