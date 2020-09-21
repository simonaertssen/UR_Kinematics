import numpy as np
import time


def T(d, theta, r, alpha):
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
    
    def __init__(self):
        super(ForwardKinematics).__init__()
        self.shoulder = T(0.089159, 0, 0.134, np.pi / 2)
        self.elbow    = T(0, -np.pi / 2, -0.425, 0)
        self.elbow2   = T(0.119, 0, 0, 0)
        Shoulder[0, 3], Shoulder[1, 3] = -Shoulder[1, 3], Shoulder[0, 3]


        # Elbow    = T(0, theta_shoulder, 0.425, 0)
        Elbow = Shoulder @ Elbow

        Elbow2 = T(0.119, 0, 0, 0)
        # Elbow2   = T(0.119, theta_elbow, 0, 0)
        Elbow2 = Elbow @ Elbow2

        # Wrist1   = T(0, -np.pi/2, 0.39225, 0)
        Wrist1 = T(0, 0, -0.39225, 0)
        Wrist1 = Elbow2 @ Wrist1

        Wrist2 = T(0.09475, -np.pi / 2, 0, np.pi / 2)
        Wrist2 = Wrist1 @ Wrist2

        Wrist3 = T(0.09475, 0, 0, -np.pi / 2)
        Wrist3 = Wrist2 @ Wrist3

        Tool = T(0.0815, 0, 0, 0)
        Tool = Wrist3 @ Tool
        d = [0.089159]
        r = []
        alpha = []


def testKinematicsSpeed():
    pass

if __name__ == '__main__':
    testKinematicsSpeed()