
# cdef extern double forwardkinematics(int)
#
# cpdef ForwardKinematics(joint_angles, tool_position=None):
#     a, b, c, d, e, f = np.float64(joint_angles)
#     if tool_position is not None:
#         x, y, z = tool_position
#     else:
#         x, y, z = np.nan, np.nan, np.nan


cpdef ForwardKinematics(joint_angles, tool_position=None):
    print('Test')
    return 1
