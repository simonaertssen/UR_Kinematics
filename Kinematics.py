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


class ForwardKinematics(object):
    # Use slots for faster access:
    # https://stackoverflow.com/questions/472000/usage-of-slots
    __slots__ = ('base', 'shoulder', 'elbow', 'elbowend', 'wrist1', 'wrist2', 'wrist3')

    def __init__(self):
        super(ForwardKinematics).__init__()
        self.base     = np.zeros((4, 4))
        self.shoulder = np.zeros((4, 4))
        self.elbow    = np.zeros((4, 4))
        self.elbowend = np.zeros((4, 4))
        self.wrist1   = np.zeros((4, 4))
        self.wrist2   = np.zeros((4, 4))
        self.wrist3   = np.zeros((4, 4))

    def positions(self):
        X = np.array([0,               0, self.base[0, 3], self.shoulder[0, 3], self.elbow[0, 3], self.elbowend[0, 3], self.wrist1[0, 3], self.wrist2[0, 3], self.wrist3[0, 3]])
        Y = np.array([0,               0, self.base[1, 3], self.shoulder[1, 3], self.elbow[1, 3], self.elbowend[1, 3], self.wrist1[1, 3], self.wrist2[1, 3], self.wrist3[1, 3]])
        Z = np.array([0, self.base[2, 3], self.base[2, 3], self.shoulder[2, 3], self.elbow[2, 3], self.elbowend[2, 3], self.wrist1[2, 3], self.wrist2[2, 3], self.wrist3[2, 3]])
        return X, Y, Z

    def forward(self, jointangles):
        a, b, c, d, e, f = jointangles
        # The joint parameters a, d and alpha can be found here: https://www.universal-robots.com/articles/ur-articles/parameters-for-calculations-of-kinematics-and-dynamics/
        self.base     = T(theta=a, d=0.089159,  r=0.134,    alpha=np.pi / 2)
        self.shoulder = T(theta=b, d=0,         r=-0.425,   alpha=0)
        self.elbow    = T(theta=c, d=0.119,     r=0,        alpha=0)
        self.elbowend = T(theta=c, d=0,         r=-0.39225, alpha=0)
        self.wrist1   = T(theta=d, d=-0.09475,  r=0,        alpha=np.pi / 2)
        self.wrist2   = T(theta=e, d=0.09475,   r=0,        alpha=-np.pi / 2)
        self.wrist3   = T(theta=f, d=-0.0815,   r=0,        alpha=0)

        self.base[0, 3], self.base[1, 3] = -self.base[1, 3], self.base[0, 3]
        self.shoulder = self.base.dot(self.shoulder)
        self.elbow    = self.shoulder.dot(self.elbow)
        self.elbowend = self.elbow.dot(self.elbowend)
        self.wrist1   = self.elbowend.dot(self.wrist1)
        self.wrist2   = self.wrist1.dot(self.wrist2)
        self.wrist3   = self.wrist2.dot(self.wrist3)

        return self.positions()


def SpeedOfCurrentKinematics():
    fk = ForwardKinematics()
    start = time.time()
    for _ in range(100000):
        pos = fk.forward([0, -np.pi/2, 0, -np.pi/2, 0, 0])
    interval = time.time() - start
    print(100000/interval, "iterations per second")


if __name__ == '__main__':
    SpeedOfCurrentKinematics()
    # 17945 iterations per second without 'slots'
    # 17859 iterations per second with 'slots'
    # 17663 iterations per second without for loop
