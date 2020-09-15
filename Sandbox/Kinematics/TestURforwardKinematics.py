import numpy as np
import time

def HTM(i, theta):
    """Calculate the HTM between two links.
    Args:
        i: A target index of joint value.
        theta: A list of joint value solution. (unit: radian)
    Returns:
        An HTM of Link l w.r.t. Link l-1, where l = i + 1.
    """

    Rot_z = np.array(np.identity(4))
    Rot_z[0, 0] = Rot_z[1, 1] = np.cos(theta[i])
    Rot_z[0, 1] = -np.sin(theta[i])
    Rot_z[1, 0] = np.sin(theta[i])

    Trans_z = np.array(np.identity(4))
    Trans_z[2, 3] = d[i]

    Trans_x = np.array(np.identity(4))
    Trans_x[0, 3] = a[i]

    Rot_x = np.array(np.identity(4))
    Rot_x[1, 1] = Rot_x[2, 2] = np.cos(alpha[i])
    Rot_x[1, 2] = -np.sin(alpha[i])
    Rot_x[2, 1] = np.sin(alpha[i])

    A_i = Rot_z @ Trans_z @ Trans_x @ Rot_x
    theta = theta[i]
    cost = np.cos(theta)
    sint = np.sin(theta)
    cosa = np.cos(alpha[i])
    sina = np.sin(alpha[i])

    T = np.array([[cost, -sint * cosa, sint * sina, a[i]*cost],
                  [sint, cost * cosa, -cost * sina, a[i]*sint],
                  [0, sina, cosa, d[i]],
                  [0, 0, 0, 1]])
    return A_i


d1 = 0.089159
d2 = d3 = 0
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823

# a (unit: mm)
a1 = a4 = a5 = a6 = 0
a2 = -0.425
a3 = -0.39225

d = np.array([d1, d2, d3, d4, d5, d6])
a = np.array([a1, a2, a3, a4, a5, a6])
alpha = np.array([0, -np.pi/2, 0, -np.pi/2, 0, 0])

# {'base': RobotJoint(0.0), 'shoulder': RobotJoint(-1.5707774718194822), 'elbow': RobotJoint(0.0), 'wrist1': RobotJoint(-1.570796327),
# 'wrist2': RobotJoint(-1.570796327), 'wrist3': RobotJoint(-1.570796327)

theta = np.array([0, -np.pi/2, 0, -np.pi/2, 0, 0])

T_06 = np.array(np.identity(4))

start = time.time()
for i in range(6):
    T_06 = T_06 @ HTM(i, theta)
    print(round(T_06[0, 3], 2), round(T_06[1, 3], 2), round(T_06[2, 3], 2))

print(time.time() - start)