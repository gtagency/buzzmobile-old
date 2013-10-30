#include <stdio.h>

void get_edges(double *points_x, double *points_y,
               double *edges1_x, double *edges1_y,
               double *edges2_x, double *edges2_y,
               int length, int *num_edges);

void get_edges(double *points_x, double *points_y,
               double *edges1_x, double *edges1_y,
               double *edges2_x, double *edges2_y,
               int length, int *num_edges){
    int i,j;
    int n = 0;
    double d, dx, dy;
    d = 1;

    for (i = 0; i < length; i++){
        for (j = i+1; j < length; j++){
            dx = points_x[j] - points_x[i];
            dy = points_y[j] - points_y[i];
            if((dx*dx) + (dy*dy) <= d){
                edges1_x[n] = points_x[i];
                edges1_y[n] = points_y[i];
                edges2_x[n] = points_x[j];
                edges2_y[n] = points_y[j];
                n++;
            }
        }
    }
    *num_edges = n;
}