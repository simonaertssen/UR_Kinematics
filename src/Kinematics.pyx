from libc.math cimport sin, cos
from libc.stdio cimport printf
from libc.stdlib cimport calloc, free

# cdef extern from "forwardkinematics.h":
#     double *makeT(double, double, double, double)
#     void matmul(double*, double*)
#     double *forwardkinematics_c(double, double, double, double, double, double, short, double, double, double)


cdef double *T(double theta, double d, double r, double alpha):
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
    # Multipliy matrices A and B in 1D format and store result in B.
    cdef double *result = <double*> calloc(16, sizeof(double))
    result[0]  = A[0]*B[0] + A[1]*B[4] + A[2]*B[8] + A[3]*B[12]
    result[1]  = A[0]*B[1] + A[1]*B[5] + A[2]*B[9] + A[3]*B[13]
    result[2]  = A[0]*B[2] + A[1]*B[6] + A[2]*B[10] + A[3]*B[14]
    result[3]  = A[0]*B[3] + A[1]*B[7] + A[2]*B[11] + A[3]*B[15]
    # for i in range(4):
    #     A[i] = result[i]

    result[4]  = A[4]*B[0] + A[5]*B[4] + A[6]*B[8] + A[7]*B[12]
    result[5]  = A[4]*B[1] + A[5]*B[5] + A[6]*B[9] + A[7]*B[13]
    result[6]  = A[4]*B[2] + A[5]*B[6] + A[6]*B[10] + A[7]*B[14]
    result[7]  = A[4]*B[3] + A[5]*B[7] + A[6]*B[11] + A[7]*B[15]
    # for i in range(4):
    #     A[i+4] = result[i]

    result[8]  = A[8]*B[0] + A[9]*B[4] + A[10]*B[8] + A[11]*B[12]
    result[9]  = A[8]*B[1] + A[9]*B[5] + A[10]*B[9] + A[11]*B[13]
    result[10] = A[8]*B[2] + A[9]*B[6] + A[10]*B[10] + A[11]*B[14]
    result[11] = A[8]*B[3] + A[9]*B[7] + A[10]*B[11] + A[11]*B[15]
    # for i in range(4):
    #     A[i + 8] = result[i]

    result[12] = A[12]*B[0] + A[13]*B[4] + A[14]*B[8] + A[15]*B[12]
    result[13] = A[12]*B[1] + A[13]*B[5] + A[14]*B[9] + A[15]*B[13]
    result[14] = A[12]*B[2] + A[13]*B[6] + A[14]*B[10] + A[15]*B[14]
    result[15] = A[12]*B[3] + A[13]*B[7] + A[14]*B[11] + A[15]*B[15]

    cdef int i
    for i in range(16):
        B[i] = result[i]
    free(result)


cdef void printarray(double* array):
    # Print a 4 x 4 array
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
    # Matrix multiplication is correct.
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
    cdef double pihalf = 1.57079632679

    cdef double a, b, c, d, e, f, x, y, z
    a, b, c, d, e, f = joint_angles

    # cdef double *base     = <double*> makeT(a,  0.089159, -0.134,    pihalf)
    # cdef double *shoulder = <double*> makeT(b,  0,        -0.425,    0)
    # cdef double *elbow    = <double*> makeT(c, -0.119,     0,        0)
    # cdef double *elbowend = <double*> makeT(0,  0,        -0.39225,  0)
    # cdef double *wrist1   = <double*> makeT(d,  0.09475,   0,        pihalf)
    # cdef double *wrist2   = <double*> makeT(e,  0.09475,   0,       -pihalf)
    # cdef double *wrist3   = <double*> makeT(f,  0.0815,    0,        0)

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

    # test()

    # printarray(base)
    # printarray(shoulder)
    # printarray(elbow)
    # printarray(elbowend)
    # printarray(wrist1)
    # printarray(wrist2)
    # printarray(wrist3)

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


# cdef forwardkinematics(joint_angles, tool_position=None):
#     cdef double a, b, c, d, e, f, x, y, z
#     a, b, c, d, e, f = joint_angles
#     cdef short i = 9
#     if tool_position is not None:
#         i += 1
#         x, y, z = tool_position
#     else:
#         x = -1000
#         y = -1000
#         z = -1000
#
#     cdef double* results = <double*>forwardkinematics_c(a, b, c, d, e, f, i, x, y, z)
#     cdef list X, Y, Z
#     X = [float(results[i]) for i in range(0,9)]
#     Y = [float(results[i]) for i in range(10,18)]
#     Z = [float(results[i]) for i in range(19,27)]
#     return X, Y, Z

cpdef ForwardKinematics(joint_angles, tool_position=None):
    return forwardkinematics(joint_angles, tool_position=tool_position)


