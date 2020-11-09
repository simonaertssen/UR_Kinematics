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