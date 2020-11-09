from libc.math cimport sin, cos
from libc.stdlib cimport calloc, free

cdef extern from "forwardkinematics.h":
    double *makeT(double, double, double, double)
    void matmul(double*, double*)

cdef double *T(double theta, double d, double r, double alpha):
    cos_t = cos(theta)
    sin_t = sin(theta)
    cos_a = cos(alpha)
    sin_a = sin(alpha)
    cdef double *result = <double*> calloc(16, sizeof(double))

    result[0], result[1], result[2], result[3] = cos_t, -sin_t*cos_a,  sin_t*sin_a, r*cos_t
    result[4], result[5], result[6], result[7] = sin_t,  cos_t*cos_a, -cos_t*sin_a, r*sin_t
    result[9], result[10], result[11] = sin_a, cos_a, d
    result[16] = 1
    return result

cdef void dot(double *A, double *B):
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
    A = result
    free(result)

cdef forwardkinematics(joint_angles, tool_position=None):
    cdef double pihalf = 1.57079632679

    cdef double a, b, c, d, e, f, x, y, z
    a, b, c, d, e, f = joint_angles

    cdef double *base     = T(theta=a, d=0.089159, r=-0.134,   alpha=pihalf)
    cdef double *shoulder = T(theta=b, d=0,        r=-0.425,   alpha=0)
    cdef double *elbow    = T(theta=c, d=-0.119,   r=0,        alpha=0)
    cdef double *elbowend = T(theta=0, d=0,        r=-0.39225, alpha=0)
    cdef double *wrist1   = T(theta=d, d=0.09475,  r=0,        alpha=pihalf)
    cdef double *wrist2   = T(theta=e, d=0.09475,  r=0,        alpha=-pihalf)
    cdef double *wrist3   = T(theta=f, d=0.0815,   r=0,        alpha=0)

    base[3], base[7] = -base[7], base[3]
    matmul(shoulder, base)
    matmul(elbow, shoulder)
    matmul(elbowend, elbow)
    matmul(wrist1, elbowend)
    matmul(wrist2, wrist1)
    matmul(wrist3, wrist2)

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
    return forwardkinematics(joint_angles, tool_position=tool_position)


