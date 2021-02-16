from libc.math cimport sin, cos
from libc.stdio cimport printf
from libc.stdlib cimport calloc, free

cdef extern from "forwardkinematics.h":
    double *T_c(double, double, double, double)
    void dot_c(double*, double*)


cdef double *T(double theta, double d, double r, double alpha):
    """
    Make a rotation and translation matrix according to the
    Denavit-Hartenberg convention. Replaced by a faster C implementation.

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
    result : c double array pointer
        1 x 16 matrix containing the rotation and translation.
    """

    cdef double cos_t = cos(theta)
    cdef double sin_t = sin(theta)
    cdef double cos_a = cos(alpha)
    cdef double sin_a = sin(alpha)
    cdef double *result = <double*> calloc(16, sizeof(double))

    result[0], result[1], result[2], result[3] = cos_t, -sin_t*cos_a,  sin_t*sin_a, r*cos_t
    result[4], result[5], result[6], result[7] = sin_t,  cos_t*cos_a, -cos_t*sin_a, r*sin_t
    result[9], result[10], result[11] = sin_a, cos_a, d
    result[15] = 1
    return result


cdef void dot(double *A, double *B):
    """
    Multiply two matrices that represent a 4 x 4 matrix in 1 x 16 format. Unroll
    loops and store results in B. Replaced by a faster C implementation.

    Parameters:
    ----------
    A, B : c double array pointer, c double array pointer
      The two 1 x 16 double arrays we wish to multiply.

    Returns:
    ----------
    B : c double array pointer
        1 x 16 matrix containing matrix multiplication.
    """

    cdef double *result = <double*> calloc(16, sizeof(double))
    result[0]  = A[0]*B[0] + A[1]*B[4] + A[2]*B[8] + A[3]*B[12]
    result[1]  = A[0]*B[1] + A[1]*B[5] + A[2]*B[9] + A[3]*B[13]
    result[2]  = A[0]*B[2] + A[1]*B[6] + A[2]*B[10] + A[3]*B[14]
    result[3]  = A[0]*B[3] + A[1]*B[7] + A[2]*B[11] + A[3]*B[15]

    result[4]  = A[4]*B[0] + A[5]*B[4] + A[6]*B[8] + A[7]*B[12]
    result[5]  = A[4]*B[1] + A[5]*B[5] + A[6]*B[9] + A[7]*B[13]
    result[6]  = A[4]*B[2] + A[5]*B[6] + A[6]*B[10] + A[7]*B[14]
    result[7]  = A[4]*B[3] + A[5]*B[7] + A[6]*B[11] + A[7]*B[15]

    result[8]  = A[8]*B[0] + A[9]*B[4] + A[10]*B[8] + A[11]*B[12]
    result[9]  = A[8]*B[1] + A[9]*B[5] + A[10]*B[9] + A[11]*B[13]
    result[10] = A[8]*B[2] + A[9]*B[6] + A[10]*B[10] + A[11]*B[14]
    result[11] = A[8]*B[3] + A[9]*B[7] + A[10]*B[11] + A[11]*B[15]

    result[12] = A[12]*B[0] + A[13]*B[4] + A[14]*B[8] + A[15]*B[12]
    result[13] = A[12]*B[1] + A[13]*B[5] + A[14]*B[9] + A[15]*B[13]
    result[14] = A[12]*B[2] + A[13]*B[6] + A[14]*B[10] + A[15]*B[14]
    result[15] = A[12]*B[3] + A[13]*B[7] + A[14]*B[11] + A[15]*B[15]

    cdef int i
    for i in range(16):
        B[i] = result[i]
    free(result)


cdef void printarray(double* array):
    """
    Print the contents of 1 x 16 array for inspection.
    """
    cdef int i
    printf("Array = \n[")
    for i in range(4):
        if i != 0:
            printf(" ")
        printf("[")

        for j in range(4):
            printf("%10.3lf", array[i*4 + j])
            if j < 3:
                printf(" ")

        printf("]")
        if i != 3:
            printf("\n")
    printf("]\n")


cdef void test():
    """
    Test whether matrix multiplication in dot() is correct.
    """
    cdef double * test1 = < double * > calloc(16, sizeof(double))
    test1[0] = -2.1
    test1[5] = -2.1
    test1[10] = -2.1
    test1[15] = -2.1

    cdef double *test2 = <double*> calloc(16, sizeof(double))
    cdef int i
    for i in range(16):
        test2[i] = i+1

    printarray(test2)
    dot(test1, test2)
    printarray(test2)
    # Result should be:
    # array([[ -2.1,  -4.2,  -6.3,  -8.4],
    #        [-10.5, -12.6, -14.7, -16.8],
    #        [-18.9, -21. , -23.1, -25.2],
    #        [-27.3, -29.4, -31.5, -33.6]])


cdef forwardkinematics(joint_angles, tool_position=None):
    """
    Compute the forward kinematics of the UR5 robot arm according to the
    Denavit-Hartenberg convention. This requires a matrix per joint.
    Replaced by a faster C implementation.

    Parameters:
    ----------
    joint_angles : list
        The list of all joint angles of the UR5 robot.
    tool_position : list
        The list containing the toolhead position. Better measure than compute
        it ourselves. Optional.

    Returns:
    ----------
    X, Y, Z : list, list, list
        The lists containing all x-, y- and z-positions of all joints.
    """

    cdef double pihalf = 1.57079632679
    cdef double a, b, c, d, e, f, x, y, z
    a, b, c, d, e, f = joint_angles

    # The joint parameters a, d and alpha can be found here:
    # https://www.universal-robots.com/articles/ur-articles/parameters-for-calculations-of-kinematics-and-dynamics/
    cdef double *base     = T(a,  0.089159, -0.134,    pihalf)
    cdef double *shoulder = T(b,  0,        -0.425,    0)
    cdef double *elbow    = T(c, -0.119,     0,        0)
    cdef double *elbowend = T(0,  0,        -0.39225,  0)
    cdef double *wrist1   = T(d,  0.09475,   0,        pihalf)
    cdef double *wrist2   = T(e,  0.09475,   0,       -pihalf)
    cdef double *wrist3   = T(f,  0.0815,    0,        0)

    base[3], base[7] = -base[7], base[3]
    dot(base, shoulder)
    dot(shoulder, elbow)
    dot(elbow, elbowend)
    dot(elbowend, wrist1)
    dot(wrist1, wrist2)
    dot(wrist2, wrist3)

    cdef list X, Y, Z
    X = [0, 0, base[3], shoulder[3], elbow[3], elbowend[3], wrist1[3], wrist2[3], wrist3[3]]
    Y = [0, 0, base[7], shoulder[7], elbow[7], elbowend[7], wrist1[7], wrist2[7], wrist3[7]]
    Z = [0, base[11], base[11], shoulder[11], elbow[11], elbowend[11], wrist1[11], wrist2[11], wrist3[11]]

    free(base)
    free(shoulder)
    free(elbow)
    free(elbowend)
    free(wrist1)
    free(wrist2)
    free(wrist3)

    if tool_position is not None:
        x, y, z = tool_position
        X.append(x)
        Y.append(y)
        Z.append(z)

    return X, Y, Z


cdef forwardkinematics_fromc(joint_angles, tool_position=None):
    """
    Compute the forward kinematics of the UR5 robot arm according to the
    Denavit-Hartenberg convention. This requires a matrix per joint.
    Fast C implementation.

    Parameters:
    ----------
    joint_angles : list
        The list of all joint angles of the UR5 robot.
    tool_position : list
        The list containing the toolhead position. Better measure than compute
        it ourselves. Optional.

    Returns:
    ----------
    X, Y, Z : list, list, list
        The lists containing all x-, y- and z-positions of all joints.
    """

    cdef double pihalf = 1.57079632679
    cdef double a, b, c, d, e, f, x, y, z
    a, b, c, d, e, f = joint_angles

    # The joint parameters a, d and alpha can be found here:
    # https://www.universal-robots.com/articles/ur-articles/parameters-for-calculations-of-kinematics-and-dynamics/
    cdef double *base     = <double*> T_c(a,  0.089159, -0.134,    pihalf)
    cdef double *shoulder = <double*> T_c(b,  0,        -0.425,    0)
    cdef double *elbow    = <double*> T_c(c, -0.119,     0,        0)
    cdef double *elbowend = <double*> T_c(0,  0,        -0.39225,  0)
    cdef double *wrist1   = <double*> T_c(d,  0.09475,   0,        pihalf)
    cdef double *wrist2   = <double*> T_c(e,  0.09475,   0,       -pihalf)
    cdef double *wrist3   = <double*> T_c(f,  0.0815,    0,        0)

    base[3], base[7] = -base[7], base[3]
    dot_c(base, shoulder)
    dot_c(shoulder, elbow)
    dot_c(elbow, elbowend)
    dot_c(elbowend, wrist1)
    dot_c(wrist1, wrist2)
    dot_c(wrist2, wrist3)

    cdef list X, Y, Z
    X = [0, 0, base[3], shoulder[3], elbow[3], elbowend[3], wrist1[3], wrist2[3], wrist3[3]]
    Y = [0, 0, base[7], shoulder[7], elbow[7], elbowend[7], wrist1[7], wrist2[7], wrist3[7]]
    Z = [0, base[11], base[11], shoulder[11], elbow[11], elbowend[11], wrist1[11], wrist2[11], wrist3[11]]

    free(base)
    free(shoulder)
    free(elbow)
    free(elbowend)
    free(wrist1)
    free(wrist2)
    free(wrist3)

    if tool_position is not None:
        x, y, z = tool_position
        X.append(x)
        Y.append(y)
        Z.append(z)

    return X, Y, Z


cpdef ForwardKinematics(joint_angles, tool_position=None):
    """
    Call the fast C implementation straignt from Python.
    """
    return forwardkinematics_fromc(joint_angles, tool_position=tool_position)


cpdef detectCollision(positions):
    """
    Detect whether any of the spatial positions computed by the forward kinematics
    is out of bounds. This prevents the robot arm from bumping into the container,
    or into the cameras or the screen. Add a margin e for security.
    Fast Cython implementation.

    Parameters:
    ----------
    positions : tuple of lists
        The lists containing all x-, y- and z-positions of all joints.

    Returns:
    ----------
    bool
        The boolean whether we detected a collision (True) or not (False).
    """

    cdef list X, Y, Z
    X, Y, Z = positions
    X = X[2:]  # We don't need all the positions
    Y = Y[2:]
    Z = Z[2:]
    cdef size_t items = len(X)

    # Are we inside of the box?
    cdef double e = 0.05
    cdef double BOX_X_MIN = -0.832
    cdef double BOX_X_MAX = 0.490
    cdef double BOX_Y_MIN = -0.713
    cdef double BOX_Y_MAX = 0.265
    cdef double BOX_Z_MIN = 0

    cdef int i = 0
    for i in range(items):
      if not (((BOX_X_MIN + e < X[i]) & (X[i] < BOX_X_MAX - e)) and ((BOX_Y_MIN + e < Y[i]) & (Y[i] < BOX_Y_MAX - e)) and (BOX_Z_MIN < Z[i])):
        print("Kinematics.pyx: you are about to hit the box")
        return True
    # Are we bumping into the camera and the light?
    CAM_X_MIN = -0.568
    CAM_X_MAX = -0.364
    CAM_Y_MIN = -0.266
    CAM_Y_MAX = 0.031
    CAM_Z_MIN = 0.790
    for i in range(items):
      if ((CAM_X_MIN + e < X[i]) & (X[i] < CAM_X_MAX - e)) and ((CAM_Y_MIN + e < Y[i]) & (Y[i] < CAM_Y_MAX - e)) and (CAM_Z_MIN + e < Z[i]):
        print("Kinematics.pyx: you are about to hit the camera")
        return True
    # Are we bumping into the screen?
    e = 0.01
    SCR_X_MAX = -0.258
    SCR_Y_MAX = -0.525
    SCR_Z_MIN = 0.400
    for i in range(items):
      if (X[i] < SCR_X_MAX + e) and (Y[i] < SCR_Y_MAX + e) and (Z[i] < SCR_Z_MIN + e):
        print("Kinematics.pyx: you are about to hit the screen")
        return True
    return False
