#include <stdio.h>

void get_edges(double *x1, double *y1,
               double *x2, double *y2,
               double *x1out, double *y1out,
               double *x2out, double *y2out,
               int length);

void get_edges(double *x1, double *y1,
               double *x2, double *y2,
               double *x1out, double *y1out,
               double *x2out, double *y2out,
               int length){
    int i,j;
    int n = 0;
    double d = 1;

    for (i = 0; i < length; i++){
        for (j = i+1; j < length; j++){
            if(((x2[i]-x1[i])*(x2[i]-x1[i]) + (y2[i]-y1[i])*(y2[i]-y1[i])) <= d){
                x1out[n] = x1[i];
                y1out[n] = y1[j];
                x2out[n] = x2[i];
                y2out[n] = y2[j];
                n++;
            }
        }
    }
}