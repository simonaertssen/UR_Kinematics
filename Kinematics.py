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
    __slots__ = ('shoulder', 'elbow', 'elbow2', 'wrist1', 'wrist2', 'wrist3', 'tool')

    def __init__(self):
        super(ForwardKinematics).__init__()
        self.shoulder = np.zeros((4, 4))
        self.elbow    = np.zeros((4, 4))
        self.elbow2   = np.zeros((4, 4))
        self.wrist1   = np.zeros((4, 4))
        self.wrist2   = np.zeros((4, 4))
        self.wrist3   = np.zeros((4, 4))
        self.tool     = np.zeros((4, 4))

    def setJointAngles(self, jointangles):
        a, b, c, d, e, f = jointangles
        self.shoulder = T(a, 0.089159, 0.134, np.pi / 2)
        self.elbow    = T(b, 0, -0.425, 0)
        self.elbow2   = T(b, 0.119, 0, 0)
        self.wrist1   = T(c, 0, -0.39225, 0)
        self.wrist2   = T(d, 0.09475, 0, np.pi / 2)
        self.wrist3   = T(e, 0.09475, 0, -np.pi / 2)
        self.tool     = T(f, 0.0815, 0, 0)

    def forward(self, angles):
        self.shoulder[0, 3], self.shoulder[1, 3] = -self.shoulder[1, 3], self.shoulder[0, 3]
        self.shoulder = self.shoulder.dot(self.elbow)
        self.elbow = self.elbow.dot(self.elbow)
        self.elbow2 = self.elbow2.dot(self.elbow2)
        self.wrist1 = self.wrist1.dot(self.wrist1)
        self.wrist2 = self.wrist2.dot(self.wrist2)
        self.wrist3 = self.wrist2.dot(self.wrist3)
        self.tool = self.wrist3.dot(self.tool)


        # Elbow    = T(0, theta_shoulder, 0.425, 0)
        Elbow = Shoulder @ Elbow

        # Elbow2   = T(0.119, theta_elbow, 0, 0)
        Elbow2 = Elbow @ Elbow2

        # Wrist1   = T(0, -np.pi/2, 0.39225, 0)
        Wrist1 = Elbow2 @ Wrist1


        Wrist2 = Wrist1 @ Wrist2


        Wrist3 = Wrist2 @ Wrist3


        Tool = Wrist3 @ Tool
        d = [0.089159]
        r = []
        alpha = []


def testKinematicsSpeed():
    pass

if __name__ == '__main__':
    testKinematicsSpeed()