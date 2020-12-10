import numpy as np
import math
import cv2


def RPY2rotvec(roll, pitch, yaw):
    '''
    DESCRIPTION: Convert roll pitch, yaw to to rotation vector.
    :param roll: x-rotation in radians
    :param pitch: y-rotation in radians
    :param yaw: z-rotation in radians
    :return rx: converted roll in euler angles
    :return ry: converted pitch in euler angles
    :return rz: converted yaw in euler angles
    '''
    yawMatrix = np.matrix([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitchMatrix = np.matrix([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    rollMatrix = np.matrix([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R = yawMatrix * pitchMatrix * rollMatrix

    rot = np.zeros((3, 1))
    cv2.Rodrigues(R, rot)

    rx = rot[0, 0]
    ry = rot[1, 0]
    rz = rot[2, 0]

    return rx, ry, rz


def RodriguesOld(matrix, vector):
    vr_desired = np.array([1, 0, 0])
    theta = np.arccos(np.dot(vector, vr_desired) / np.linalg.norm(vector) * np.linalg.norm(vr_desired))
    k = np.cross(vector, vr_desired) / np.linalg.norm(np.cross(vector, vr_desired))
    return vector * np.cos(theta) + (np.cross(k, vector) * np.sin(theta)) + k * (np.dot(k, vector)) * (1.0 - np.cos(theta))


def Rodrigues(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


def RPY2RotVec(roll, pitch, yaw):
    """
    DESCRIPTION: Convert roll pitch, yaw angles to a rotation vector.
    :param roll: x-rotation in radians
    :param pitch: y-rotation in radians
    :param yaw: z-rotation in radians
    :return rx: converted roll in Euler angles
    :return ry: converted pitch in Euler angles
    :return rz: converted yaw in Euler angles
    See: https://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/python-code-example-of-converting-rpyeuler-angles-to-rotation-vectorangle-axis-for-universal-robots/
    """
    sin = np.sin
    cos = np.cos

    yawMatrix = np.array([
        [cos(yaw), -sin(yaw), 0],
        [sin(yaw), cos(yaw), 0],
        [0, 0, 1]
    ])

    pitchMatrix = np.array([
        [cos(pitch), 0, sin(pitch)],
        [0, 1, 0],
        [-sin(pitch), 0, cos(pitch)]
    ])

    rollMatrix = np.array([
        [1, 0, 0],
        [0, cos(roll), -sin(roll)],
        [0, sin(roll), cos(roll)]
    ])

    RotMat = yawMatrix.dot(pitchMatrix.dot(rollMatrix))
    # Convert to rotation vector, account for special case of sin = 0
    theta = np.arccos(((RotMat[0, 0] + RotMat[1, 1] + RotMat[2, 2]) - 1) / 2)
    sin_theta = math.sin(theta)
    if sin_theta == 0:
        rx, ry, rz = 0.0, 0.0, 0.0
    else:
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (RotMat[2, 1] - RotMat[1, 2]) * theta
        ry = multi * (RotMat[0, 2] - RotMat[2, 0]) * theta
        rz = multi * (RotMat[1, 0] - RotMat[0, 1]) * theta
    return rx, ry, rz


if __name__ == '__main__':
    # Test Mads' implementation:
    RX = 0.0
    RY = np.pi
    RZ = (180 - 46) * np.pi / 180

    # [ 2.892 -1.228 -0.   ]
    print(np.round(RPY2rotvec(RX, RY, RZ), 3))
    print(np.round(RPY2RotVec(RX, RY, RZ), 3))
