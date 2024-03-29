import numpy as np
import time

import cv2  # For Rodrigues


def T(theta, d, r, alpha):
    """
    Make a rotation and translation matrix according to the
    Denavit-Hartenberg convention. Replaced by a faster Cython implementation.

    Parameters:
    ----------
    theta : float
        The angle at which the joint is rotated with respect to the previous arm.
    d : float
        The distance between the previous x-axis and the current x-axis, along
        the previous z-axis.
    r : float
        The length of the common normal, which is the distance between the
        previous z-axis and the current z-axis.
    alpha : float
        The angle around the common normal to between the previous z-axis and
        the current z-axis.

    Returns:
    ----------
    result : np.array
        4 x 4 matrix containing the rotation and translation.
    """

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    cos_a = np.cos(alpha)
    sin_a = np.sin(alpha)
    result = np.zeros((4, 4), np.float64)
    result[0, 0], result[0, 1], result[0, 2], result[0, 3] = cos_t, -sin_t*cos_a,  sin_t*sin_a, r*cos_t
    result[1, 0], result[1, 1], result[1, 2], result[1, 3] = sin_t,  cos_t*cos_a, -cos_t*sin_a, r*sin_t
    result[2, 1], result[2, 2], result[2, 3] = sin_a, cos_a, d
    result[3, 3] = 1
    return result


def ForwardKinematics(joint_angles, tool_position=None):
    """
    Compute the forward kinematics of the UR5 robot arm according to the
    Denavit-Hartenberg convention. This requires a matrix per joint.
    Replaced by a faster Cython implementation.

    Parameters:
    ----------
    joint_angles : list
        The list of all joint angles of the UR5 robot.
    tool_position : list
        The list containing the toolhead position. Better measure than compute
        it ourselves. Optional.

    Returns:
    ----------
    X, Y, Z : np.array, np.array, np.array
        The arrays containing all x-, y- and z-positions of all joints.
    """

    a, b, c, d, e, f = np.float64(joint_angles)
    # The joint parameters a, d and alpha can be found here:
    # https://www.universal-robots.com/articles/ur-articles/parameters-for-calculations-of-kinematics-and-dynamics/
    base     = T(theta=a, d=0.089159, r=-0.134,   alpha=np.pi / 2)
    shoulder = T(theta=b, d=0,        r=-0.425,   alpha=0)
    elbow    = T(theta=c, d=-0.119,   r=0,        alpha=0)
    elbowend = T(theta=0, d=0,        r=-0.39225, alpha=0)
    wrist1   = T(theta=d, d=0.09475,  r=0,        alpha=np.pi / 2)
    wrist2   = T(theta=e, d=0.09475,  r=0,        alpha=-np.pi / 2)
    wrist3   = T(theta=f, d=0.0815,   r=0,        alpha=0)

    base[0, 3], base[1, 3] = -base[1, 3], base[0, 3]
    shoulder = base.dot(shoulder)
    elbow = shoulder.dot(elbow)
    elbowend = elbow.dot(elbowend)
    wrist1 = elbowend.dot(wrist1)
    wrist2 = wrist1.dot(wrist2)
    wrist3 = wrist2.dot(wrist3)

    X = [0, 0, base[0, 3], shoulder[0, 3], elbow[0, 3], elbowend[0, 3], wrist1[0, 3], wrist2[0, 3], wrist3[0, 3]]
    Y = [0, 0, base[1, 3], shoulder[1, 3], elbow[1, 3], elbowend[1, 3], wrist1[1, 3], wrist2[1, 3], wrist3[1, 3]]
    Z = [0, base[2, 3], base[2, 3], shoulder[2, 3], elbow[2, 3], elbowend[2, 3], wrist1[2, 3], wrist2[2, 3], wrist3[2, 3]]

    if tool_position is not None:
        x, y, z = tool_position
        X.append(x)
        Y.append(y)
        Z.append(z)
    return np.array(X), np.array(Y), np.array(Z)


def detectCollision(positions):
    """
    Detect whether any of the spatial positions computed by the forward kinematics
    is out of bounds. This prevents the robot arm from bumping into the container,
    or into the cameras or the screen. Add a margin e for security.
    Replaced by a faster Cython implementation.

    Parameters:
    ----------
    positions : tuple of lists
        The lists containing all x-, y- and z-positions of all joints.

    Returns:
    ----------
    bool
        The boolean whether we detected a collision (True) or not (False).
    """

    X, Y, Z = positions
    X = np.array(X[2:])  # We don't need all the positions
    Y = np.array(Y[2:])
    Z = np.array(Z[2:])

    # Are we inside of the box?
    e = 0.05
    BOX_X_MIN = -0.832
    BOX_X_MAX =  0.490
    BOX_Y_MIN = -0.713
    BOX_Y_MAX =  0.265
    BOX_Z_MIN =  0.000
    if not (((BOX_X_MIN + e < X) & (X < BOX_X_MAX - e)).all() and ((BOX_Y_MIN + e < Y) & (Y < BOX_Y_MAX - e)).all() and (BOX_Z_MIN < Z).all()):
        print("box")
        return True
    # Are we bumping into the camera and the light?
    CAM_X_MIN = -0.568
    CAM_X_MAX = -0.364
    CAM_Y_MIN = -0.266
    CAM_Y_MAX =  0.031
    CAM_Z_MIN =  0.765
    if ((CAM_X_MIN + e < X) & (X < CAM_X_MAX - e)).any() and ((CAM_Y_MIN + e < Y) & (Y < CAM_Y_MAX - e)).any() and (CAM_Z_MIN + e > Z).any():
        print("camera")
        return True
    # Are we bumping into the screen?
    e = 0.01
    SCR_X_MAX = -0.182
    SCR_Y_MAX = -0.520
    SCR_Z_MIN =  0.375
    if (X < SCR_X_MAX + e).any() and (Y < SCR_Y_MAX + e).any() and (Z < SCR_Z_MIN + e).any():
        print("screen")
        return True
    return False


def RPY2RotVecRodr(roll, pitch, yaw):
    """
    Convert roll pitch, yaw angles to a rotation vector.
    See: www.zacobria.com/universal-robots-knowledge-base-tech-support-
        forum-hints-tips/python-code-example-of-converting-rpyeuler-angles-to-
        rotation-vectorangle-axis-for-universal-robots/

    Parameters:
    ----------
    roll : float
        The x-rotation in radians.
    pitch : float
        The y-rotation in radians.
    yaw : float
        The z-rotation in radians.

    Returns:
    ----------
    rx, ry, rz
        The converted roll, pitch and yaw in Euler angles
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

    rot = np.zeros((3, 1))
    cv2.Rodrigues(RotMat, rot)

    rx = rot[0, 0]
    ry = rot[1, 0]
    rz = rot[2, 0]
    return rx, ry, rz


def RPY2RotVec(gamma, beta, alpha):
    # See https://forum.universal-robots.com/t/pose-rotation-order/223/2
    ca = np.cos(alpha)
    cb = np.cos(beta)
    cg = np.cos(gamma)
    sa = np.sin(alpha)
    sb = np.sin(beta)
    sg = np.sin(gamma)

    r11 = ca * cb
    r12 = ca * sb * sg - sa * cg
    r13 = ca * sb * cg + sa * sg
    r21 = sa * cb
    r22 = sa * sb * sg + ca * cg
    r23 = sa * sb * cg - ca * sg
    r31 = -sb
    r32 = cb * sg
    r33 = cb * cg

    theta = np.arccos((r11 + r22 + r33 - 1) / 2)
    sth = np.sin(theta)
    kx = (r32 - r23) / (2 * sth)
    ky = (r13 - r31) / (2 * sth)
    kz = (r21 - r12) / (2 * sth)
    return theta * kx, theta * ky, theta * kz


def RotVec2RPY(rx, ry, rz):
    # See https://forum.universal-robots.com/t/pose-rotation-order/223/2
    theta = np.sqrt(rx * rx + ry * ry + rz * rz)
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta
    cth = np.cos(theta)
    sth = np.sin(theta)
    vth = 1 - np.cos(theta)

    r11 = kx * kx * vth + cth
    r12 = kx * ky * vth - kz * sth
    r13 = kx * kz * vth + ky * sth
    r21 = kx * ky * vth + kz * sth
    r22 = ky * ky * vth + cth
    r23 = ky * kz * vth - kx * sth
    r31 = kx * kz * vth - ky * sth
    r32 = ky * kz * vth + kx * sth
    r33 = kz * kz * vth + cth

    beta = np.arctan2(-r31, np.sqrt(r11 * r11 + r21 * r21))
    if beta > 1.570621793869697:  # beta > 89.99:
        beta = 1.570621793869697
        alpha = 0
        gamma = np.arctan2(r12, r22)
    elif beta < - 1.570621793869697:  # beta < -d2r(89.99):
        beta = -1.570621793869697
        alpha = 0
        gamma = -np.arctan2(r12, r22)
    else:
        cb = np.cos(beta)
        alpha = np.arctan2(r21 / cb, r11 / cb)
        gamma = np.arctan2(r32 / cb, r33 / cb)
    return gamma, beta, alpha


def SpeedOfCurrentKinematics():
    n = 100000
    start = time.time()
    for _ in range(n):
        toolTip = None
        pos = ForwardKinematics((0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0))
    interval = time.time() - start
    print(n/interval, "iterations per second")


def SpeedOfCollisionDetection():
    X = [0.1]*9
    Y = [0.1]*9
    Z = [0.1]*9

    n = 100000
    start = time.time()
    for _ in range(n):
        toolTip = None
        collision = detectCollision((X, Y, Z))
    interval = time.time() - start
    print(n/interval, "iterations per second")


def SpeedOfRYP():
    r = 0.0
    p = 3.141509
    y = -2.4934075403510954
    print(RotVec2RPY(r, p, y))
    print(RPY2RotVec(r, p, y))
    print(RPY2RotVecRodr(r, p, y))

    n = 100000
    start = time.time()
    for _ in range(n):
        RPY2RotVecRodr(r, p, y)
    interval = time.time() - start
    print(n / interval, "iterations per second")

    start = time.time()
    for _ in range(n):
        RPY2RotVec(r, p, y)
    interval = time.time() - start
    print(n / interval, "iterations per second")


if __name__ == '__main__':
    # SpeedOfCurrentKinematics()
    # 17945 iterations per second without 'slots'
    # 17859 iterations per second with 'slots'
    # 17663 iterations per second without for loop
    # 14195 iterations per second by using just a function

    SpeedOfCollisionDetection()
    # 47293 iterations per second for the pure Python implementation

    SpeedOfRYP()
    # Rodriguez version takes 20288 iterations per second
    # Classical version takes 46003 iterations per second
