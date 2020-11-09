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
