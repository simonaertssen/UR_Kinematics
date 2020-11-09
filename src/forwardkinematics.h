#include <stdio.h>
#include <math.h>
#include <stdlib.h>


double *makeT(double theta, double d, double r, double alpha){
    double cos_t = cos(theta);
    double sin_t = sin(theta);
    double cos_a = cos(alpha);
    double sin_a = sin(alpha);
    double *result = (double*) calloc(16, sizeof(double));

    result[0] = cos_t;
    result[1] = -sin_t*cos_a;
    result[2] = sin_t*sin_a;
    result[3] = r*cos_t;
    result[4] = sin_t;
    result[5] = cos_t*cos_a;
    result[6] = -cos_t*sin_a;
    result[7] = r*sin_t;
    result[9] = sin_a;
    result[10] = cos_a;
    result[11] = d;
    result[16] = 1;
    return result;
}

void matmul(double *A, double *B){
    short i;
    double *result = (double*) malloc(sizeof(double)*4);
    result[0]  = A[0]*B[0] + A[1]*B[4] + A[2]*B[8] + A[3]*B[12];
    result[1]  = A[0]*B[1] + A[1]*B[5] + A[2]*B[9] + A[3]*B[13];
    result[2]  = A[0]*B[2] + A[1]*B[6] + A[2]*B[10] + A[3]*B[14];
    result[3]  = A[0]*B[3] + A[1]*B[7] + A[2]*B[11] + A[3]*B[15];
    for (i = 0; i < 4; i++) A[i] = result[i];

    result[0]  = A[4]*B[0] + A[5]*B[4] + A[6]*B[8] + A[7]*B[12];
    result[1]  = A[4]*B[1] + A[5]*B[5] + A[6]*B[9] + A[7]*B[13];
    result[2]  = A[4]*B[2] + A[5]*B[6] + A[6]*B[10] + A[7]*B[14];
    result[3]  = A[4]*B[3] + A[5]*B[7] + A[6]*B[11] + A[7]*B[15];
    for (i = 0; i < 4; i++) A[i+4] = result[i];

    result[0]  = A[8]*B[0] + A[9]*B[4] + A[10]*B[8] + A[11]*B[12];
    result[1]  = A[8]*B[1] + A[9]*B[5] + A[10]*B[9] + A[11]*B[13];
    result[2] = A[8]*B[2] + A[9]*B[6] + A[10]*B[10] + A[11]*B[14];
    result[3] = A[8]*B[3] + A[9]*B[7] + A[10]*B[11] + A[11]*B[15];
    for (i = 0; i < 4; i++) A[i+8] = result[i];

    result[0] = A[12]*B[0] + A[13]*B[4] + A[14]*B[8] + A[15]*B[12];
    result[1] = A[12]*B[1] + A[13]*B[5] + A[14]*B[9] + A[15]*B[13];
    result[2] = A[12]*B[2] + A[13]*B[6] + A[14]*B[10] + A[15]*B[14];
    result[3] = A[12]*B[3] + A[13]*B[7] + A[14]*B[11] + A[15]*B[15];
    for (i = 0; i < 4; i++) A[i+12] = result[i];

    free(result);
}
//
//double *forwardkinematics_c(double a, double b, double c, double d, double e, double f, short i, double x, double y, double z){
//    short idx = 0;
//    double pi2 = 1.57079632679;
//    double *base     = makeT(a,  0.089159, -0.134,   pi2);
//    double *shoulder = makeT(b,  0,        -0.425,   0);
//    double *elbow    = makeT(c, -0.119,    0,        0);
//    double *elbowend = makeT(0,  0,        -0.39225, 0);
//    double *wrist1   = makeT(d,  0.09475,   0,       pi2);
//    double *wrist2   = makeT(e,  0.09475,   0,      -pi2);
//    double *wrist3   = makeT(f,  0.0815,    0,       0);
//
//    double tmp = base[3]; base[3] = -base[7]; base[7] = base[3];
//    matmul(shoulder, base);
//    matmul(elbow, shoulder);
//    matmul(elbowend, elbow);
//    matmul(wrist1, elbowend);
//    matmul(wrist2, wrist1);
//    matmul(wrist3, wrist2);
//
//    double *result = (double*) calloc((int) i*3, sizeof(double));
//    if (i == 10){
//        idx = 1;
//        result[9] = x;
//        result[19] = y;
//        result[29] = z;
//    }
//
//    result[2] = base[3];
//    result[3] = shoulder[3];
//    result[4] = elbow[3];
//    result[5] = elbowend[3];
//    result[6] = wrist1[3];
//    result[7] = wrist2[3];
//    result[8] = wrist3[3];
//    result[11 + idx] = base[7];
//    result[12 + idx] = shoulder[7];
//    result[13 + idx] = elbow[7];
//    result[14 + idx] = elbowend[7];
//    result[15 + idx] = wrist1[7];
//    result[16 + idx] = wrist2[7];
//    result[17 + idx] = wrist3[7];
//    result[19 + 2*idx] = base[11];
//    result[20 + 2*idx] = base[11];
//    result[21 + 2*idx] = shoulder[11];
//    result[22 + 2*idx] = elbow[11];
//    result[23 + 2*idx] = elbowend[11];
//    result[24 + 2*idx] = wrist1[11];
//    result[25 + 2*idx] = wrist2[11];
//    result[26 + 2*idx] = wrist3[11];
//
//    free(base);
//    free(shoulder);
//    free(elbow);
//    free(elbowend);
//    free(wrist1);
//    free(wrist2);
//    free(wrist3);
//
//    return result;
// }
