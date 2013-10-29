#include <stdio.h>

void get_edges(double *x, double *y,
            double *xout, double *yout,
            int length);

void get_edges(double *x, double *y,
            double *xout, double *yout,
            int length){
    int i,j;
    int n = 0;
    double d = 1;

    for (i = 0; i < length; i++){
        for (j = i+1; j < length; j++){
            if((x[i]*x[i] + y[i]*y[i]) <= d){
                xout[n] = x[i];
                yout[n] = y[j];
                n++;
            }
        }
    }
}