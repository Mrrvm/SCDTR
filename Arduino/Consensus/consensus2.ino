#include "matrixmath.h"
#define ITER_N 2

void setup() {
  Serial.begin(9600);
}

void loop() {
  // The system
  float k11 = 2, k12 = 1, k21 = 1, k22 = 2;
  float L1 = 150, L2 = 80;
  float o1 = 30, o2 = 0;
  float K[2][2] = {{k11, k12}, {k21, k22}};
  float L[2] = {L1, L2}; 
  float o[2] = {o1, o2};
  
  // The cost function
  float c1 = 1, c2 = 0; //here we have to get the c from outside, for arduino 1 (1,0), for 2 (0, 1)
  float q1 = 0.0, q2 = 0.0;
  float c[2] = {c1, c2}; 
  float Q[2][2] = {{q1, 0}, {0, q2}};
  
  // Initialization
  float rho = 0.01;
  float d[2] = {0, 0};
  float d_av[2] = {0, 0};
  float d_copy[2] = {0, 0};
  float y[2] = {0, 0};
  float min_best[50]= {0};
  float z[2] = {0, 0};
  
  // Unconstant variables
  float d1_best, d2_best;
  bool sol_unconstrained;
  bool sol_boundary_linear;
  bool sol_boundary_0;
  bool sol_boundary_100;
  bool sol_linear_0;
  bool sol_linear_100;
  float c_neg[2] = {0, 0};
  float y_neg[2] = {0, 0};
  float d_av_rho[2] = {0, 0};
  
  for(int i=1; i<ITER_N; i++) {
    
    d1_best = -1;
    d2_best = -1;
    min_best[i] = 100000;
    sol_unconstrained = 1;
    sol_boundary_linear = 1;
    sol_boundary_0 = 1;
    sol_boundary_100 = 1;
    sol_linear_0 = 1;
    sol_linear_100 = 1;
    
    Matrix.MultiplyConst((float*) c, 2, 1, -1, (float*) c_neg);
    Matrix.MultiplyConst((float*) y_neg, 2, 1, -1, (float*) y);
    Matrix.Add((float*) c_neg, (float*) y_neg, 2, 1, (float*) z);
    Matrix.Add((float*) c_neg, (float*) y_neg, 2, 1, (float*) z);
    Matrix.MultiplyConst((float*) d_av, 2, 1, rho, (float*) d_av_rho);
    Matrix.Add((float*) z, (float*) d_av_rho, 2, 1, (float*) z);
    Matrix.Print((float*) z, 2, 1, "result is:\n");
    
  }
  while(1);
  
}





















