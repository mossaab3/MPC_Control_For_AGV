#include"MPC.h"
#include<math.h>

double**  cercleRef(int max_iter, double to, double freq, double R, double Xo, double Yo, double theta0){
    double** X;
    X = new double *[3];
    for (int i = 0; i < 3; i++)
    {
        X[i] = new double[max_iter + Np];      
    }
    for (int j = 0; j <= max_iter + Np; j++)
    {
        X[0][j] = R*cos(2*M_PI*freq*j*to) + Xo;
        X[1][j] = R*sin(2*M_PI*freq*j*to) + Yo;
        X[2][j] = (freq*2*M_PI*j*to) + theta0;
    }
    return X;
}

double**  rightRef(int max_iter, double to, double a){
    double** X;
    X = new double *[3];
    for (int i = 0; i < 3; i++)
    {
        X[i] = new double[max_iter + Np];      
    }
    for (int j = 0; j < max_iter + Np; j++)
    {
        X[0][j] = j*to;
        X[1][j] = a*j*to;
        X[2][j] = atan(a);
    }
    return X;
}
