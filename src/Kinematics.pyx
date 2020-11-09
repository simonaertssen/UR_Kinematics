# import numpy as np


cpdef ForwardKinematics(joint_angles, tool_position=None):
    cdef double a, b, c, d, e, f
    a, b, c, d, e, f = joint_angles

    cdef double[::1] X, Y, Z

    # if tool_position is not None:
    #     x, y, z = tool_position
    # else:
    #     x, y, z = np.nan, np.nan, np.nan
    return a

