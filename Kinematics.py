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
    __slots__ = ('shoulder', 'elbow', 'elbowend', 'wrist1', 'wrist2', 'wrist3', 'tool')

    def __init__(self):
        super(ForwardKinematics).__init__()
        self.shoulder = np.zeros((4, 4))
        self.elbow    = np.zeros((4, 4))
        self.elbowend   = np.zeros((4, 4))
        self.wrist1   = np.zeros((4, 4))
        self.wrist2   = np.zeros((4, 4))
        self.wrist3   = np.zeros((4, 4))
        self.tool     = np.zeros((4, 4))

    def forward(self, jointangles):
        a, b, c, d, e, f = jointangles
        self.shoulder = T(a, 0.089159, 0.134, np.pi / 2)
        self.elbow    = T(b, 0, -0.425, 0)
        self.elbowend = T(b, 0.119, 0, 0)
        self.wrist1   = T(c, 0, -0.39225, 0)
        self.wrist2   = T(d, 0.09475, 0, np.pi / 2)
        self.wrist3   = T(e, 0.09475, 0, -np.pi / 2)
        self.tool     = T(f, 0.0815, 0, 0)

        self.shoulder[0, 3], self.shoulder[1, 3] = -self.shoulder[1, 3], self.shoulder[0, 3]
        self.elbow    = self.elbow.dot(self.elbow)
        self.elbowend = self.elbow.dot(self.elbowend)
        self.wrist1   = self.elbowend.dot(self.wrist1)
        self.wrist2   = self.wrist1.dot(self.wrist2)
        self.wrist3   = self.wrist2.dot(self.wrist3)
        self.tool     = self.wrist3.dot(self.tool)

    def positions(self):
        X = np.array([self.shoulder[0, 3], self.elbow[0, 3], self.elbowend[0, 3], self.wrist1[0, 3], self.wrist2[0, 3], self.wrist3[0, 3], self.tool[0, 3]])
        Y = np.array([self.shoulder[1, 3], self.elbow[1, 3], self.elbowend[1, 3], self.wrist1[1, 3], self.wrist2[1, 3], self.wrist3[1, 3], self.tool[1, 3]])
        Z = np.array([self.shoulder[2, 3], self.elbow[2, 3], self.elbowend[2, 3], self.wrist1[2, 3], self.wrist2[2, 3], self.wrist3[2, 3], self.tool[2, 3]])
        return X, Y, Z


def SpeedOfCurrentKinematics():
    fk = ForwardKinematics()
    start = time.time()
    for _ in range(100000):
        fk.forward([0, -np.pi/2, 0, -np.pi/2, 0, 0])
        pos = fk.positions()
    interval = time.time() - start
    print(100000/interval, "iterations per second")


if __name__ == '__main__':
    SpeedOfCurrentKinematics()
    # 17945 iterations per second without 'slots'
    # 17859 iterations per second with 'slots'
    # 17663 iterations per second without for loop
