#define ITER_N 2
#define NODE 0 //get this from eeprom (matlab starts at 1...)
#define N_NODES 2 

void setup() {
  Serial.begin(9600);
}

void loop() {
  // The system - this args come from outside
  float k_self = 2, k1 = 1; 
  float k[N_NODES] = {k_self, k1};
  float L = 150; 
  float o = 30; 
  
  // The cost function - this args come from outside
  float c_self = 1; 
  float q_self = 0.0;
  float c[N_NODES] = {c_self, 0}; 
  float q[N_NODES] = {q_self, 0};
  
  // Initialization 
  float rho                  = 0.01;
  float d[N_NODES]           = {0};
  float d_av[N_NODES]        = {0};
  float d_copy[N_NODES]      = {0};
  float y[N_NODES]           = {0};
  float min_best[ITER_N]     = {0};
  float z[N_NODES]           = {0};
  float p[N_NODES]           = {0};
  float d_best[N_NODES]      = {0};
  float du[N_NODES]          = {0};
  float dbl[N_NODES]         = {0};
  float db0[N_NODES]         = {0};
  float db100[N_NODES]       = {0};
  float dl100[N_NODES]       = {0};
  float dl0[N_NODES]         = {0};
  float best_d[ITER_N][NODE] = {0};
  
  bool sol_unconstrained;
  bool sol_boundary_linear;
  bool sol_boundary_0;
  bool sol_boundary_100;
  bool sol_linear_0;
  bool sol_linear_100;
  float n;
  float min_unconstrained;
  float min_boundary_linear;
  float min_boundary_0;
  float min_boundary_100;
  float min_linear_100;
  float min_linear_0;
  float w1, w2, w3; 
  float det1, det2, det3, det4;
  float x1, x2;
  float v1, v2;
  float common;
  //this will have to be with for loops
  // cuz there will be more boudaries if N_NODES is bigger...
  float u1, u2, u3;
  
  for(int i=1; i<ITER_N; i++) {
    
    d_best[NODE] = 0;
    d_best[NODE+1] = 0;
    min_best[i] = 100000;
    sol_unconstrained = 1;
    sol_boundary_linear = 1;
    sol_boundary_0 = 1;
    sol_boundary_100 = 1;
    sol_linear_0 = 1;
    sol_linear_100 = 1;
    
    z[NODE]   = -c[NODE]   + y[NODE]   + rho*d_av[NODE];
    z[NODE+1] = -y[NODE+1] + rho*d_av[NODE+1];
    
    u1 = o - L;
    u2 = 0;
    u3 = 100;

    p[NODE]   = 1/(rho*Q[1]);
    p[NODE+1] = 1/rho;

    // this must be a for loop
    n  =  k[NODE]*k[NODE]*p[NODE] + k[NODE+1]*k[NODE+1]*p[NODE+1];
    w1 = -k[NODE]*z[NODE]*p[NODE] - k[NODE+1]*z[NODE+1]*p[NODE+1];
    w2 = -z[NODE]*p[NODE];
    w3 = -(w2);
    
    // Unconstrained
    du[NODE]   = -z[NODE]*p[NODE];
    du[NODE+1] =  z[NODE+1]*p[NODE+1];
    if(du[1] < 0) { sol_unconstrained = 0;}
    if(du[1] > 100) { sol_unconstrained = 0;}
    if(k[NODE]*du[NODE] + k[NODE+1]*du[NODE+1] < L-o) { sol_unconstrained = 0;}
    if(sol_unconstrained) {
        min_unconstrained = 0.5*q[NODE]*du[NODE]^2 
                          + c[NODE]*du[NODE] 
                          + y[NODE]*(du[NODE]-d_av[NODE]) 
                          + y[NODE+1]*(du[NODE+1]-d_av[NODE+1]) 
                          + rho/(2*(du[NODE]-d_av[NODE])^2) 
                          + rho/(2*(du[NODE+1]-d_av[NODE+1])^2);
        if min_unconstrained < min_best[i] {
          d_best[NODE]   = du[NODE];
          d_best[NODE+1] = du[NODE+1];
          min_best[i] = min_unconstrained;
        }
    }

    // Boundary Linear
    dbl[NODE] = p[NODE]+z[NODE] + (p[NODE]*k[NODE])/(n*(w1-u1));
    dbl[NODE+1] = p[NODE+1]+z[NODE+1] + (p[NODE+1]*k[NODE+1])/(n*(w1-u1));
    if(dbl[NODE] < 0) {sol_boundary_linear = 0;}
    if(dbl[NODE] > 100) {sol_boundary_linear = 0;}
    if(sol_boundary_linear) {
        min_boundary_linear = = 0.5*q[NODE]*dbl[NODE]^2 
                                + c[NODE]*dbl[NODE] 
                                + y[NODE]*(dbl[NODE]-d_av[NODE]) 
                                + y[NODE+1]*(dbl[NODE+1]-d_av[NODE+1]) 
                                + rho/(2*(dbl[NODE]-d_av[NODE])^2) 
                                + rho/(2*(dbl[NODE+1]-d_av[NODE+1])^2);
        if min_boundary_linear < min_best[i] {
          d_best[NODE]   = dbl[NODE];
          d_best[NODE+1] = dbl[NODE+1];
          min_best[i] = min_boundary_linear;
        }
    }

    // Boundary 0
    db0[NODE] = 0;
    db0[NODE+1] = p[NODE+1]*z[NODE+1];
    if(db0[NODE] > 100) {sol_boundary_0 = 0;}
    if(k[NODE]*db0[NODE] + k[NODE+1]*db0[NODE+1] < L-o) {sol_boundary_0 = 0;}
    if(sol_boundary_0) {
        min_boundary_0 = = 0.5*q[NODE]*db0[NODE]^2 
                           + c[NODE]*db0[NODE] 
                           + y[NODE]*(db0[NODE]-d_av[NODE]) 
                           + y[NODE+1]*(db0[NODE+1]-d_av[NODE+1]) 
                           + rho/(2*(db0[NODE]-d_av[NODE])^2) 
                           + rho/(2*(db0[NODE+1]-d_av[NODE+1])^2);
        if min_boundary_0 < min_best[i] {
          d_best[NODE]   = db0[NODE];
          d_best[NODE+1] = db0[NODE+1];
          min_best[i] = min_boundary_0;
        }
    }

    // Boundary 100
    db100[NODE] = 0;
    db100[NODE+1] = p[NODE+1]*z[NODE+1];
    if(db100[NODE] < 0) {sol_boundary_100 = 0;}
    if(k[NODE]*db100[NODE] + k[NODE+1]*db100[NODE+1] < L-o) {sol_boundary_100 = 0;}
    if(sol_boundary_100) {
        min_boundary_100 = = 0.5*q[NODE]*db100[NODE]^2 
                             + c[NODE]*db100[NODE] 
                             + y[NODE]*(db100[NODE]-d_av[NODE]) 
                             + y[NODE+1]*(db100[NODE+1]-d_av[NODE+1]) 
                             + rho/(2*(db100[NODE]-d_av[NODE])^2) 
                             + rho/(2*(db100[NODE+1]-d_av[NODE+1])^2);
        if min_boundary_100 < min_best[i] {
          d_best[NODE]   = db100[NODE];
          d_best[NODE+1] = db100[NODE+1];
          min_best[i] = min_boundary_100;
        }
    }

    common = (rho*q[NODE])/((rho*q[NODE])*n-k[NODE]*k[NODE]);
    det1 = common;
    det2 = -k[NODE]*common;
    det3 = det2;
    det4 = n*(rho+q[NODE])*common;
    x1 = det1*w1 + det2*w2;
    x2 = det3*w1 + det4*w2;
    v1 = det1*u1 + det4*u2;
    v2 = det3*u1 + det4*u2;

    // Linear 0
    dl0[NODE]   = p[NODE]*z[NODE] + p[NODE]*k[NODE]*(x1-v1)-p[NODE]*(x2-v2);
    dl0[NODE+1] = p[NODE+1]*z[NODE+1] + p[NODE+1]*k[NODE+1]*(x1-v1);
    if(dl0[NODE] > 100) {sol_linear_0 = 0;}
    if(sol_linear_0) {
        min_linear_0 = = 0.5*q[NODE]*dl0[NODE]^2 
                         + c[NODE]*dl0[NODE] 
                         + y[NODE]*(dl0[NODE]-d_av[NODE]) 
                         + y[NODE+1]*(dl0[NODE+1]-d_av[NODE+1]) 
                         + rho/(2*(dl0[NODE]-d_av[NODE])^2) 
                         + rho/(2*(dl0[NODE+1]-d_av[NODE+1])^2);
        if min_linear_0 < min_best[i] {
          d_best[NODE]   = dl0[NODE];
          d_best[NODE+1] = dl0[NODE+1];
          min_best[i] = min_linear_0;
        }
    }

    common = (rho*q[NODE])/((rho*q[NODE])*n-k[NODE]*k[NODE]);
    det1 = common;
    det2 = -k[NODE]*common;
    det3 = det2;
    det4 = n*(rho+q[NODE])*common;
    x1 = det1*w1 + det2*w3;
    x2 = det3*w1 + det4*w3;
    v1 = det1*u1 + det4*u3;
    v2 = det3*u1 + det4*u3;

    // Linear 100
    dl100[NODE]   = p[NODE]*z[NODE] + p[NODE]*k[NODE]*(x1-v1)-p[NODE]*(x2-v2);
    dl100[NODE+1] = p[NODE+1]*z[NODE+1] + p[NODE+1]*k[NODE+1]*(x1-v1);
    if(dl100[NODE] < 0) {sol_linear_100 = 0;}
    if(sol_linear_100) {
        min_linear_100 = = 0.5*q[NODE]*dl100[NODE]^2 
                             + c[NODE]*dl100[NODE] 
                             + y[NODE]*(dl100[NODE]-d_av[NODE]) 
                             + y[NODE+1]*(dl100[NODE+1]-d_av[NODE+1]) 
                             + rho/(2*(dl100[NODE]-d_av[NODE])^2) 
                             + rho/(2*(dl100[NODE+1]-d_av[NODE+1])^2);
        if min_linear_100 < min_best[i] {
          d_best[NODE]   = dl100[NODE];
          d_best[NODE+1] = dl100[NODE+1];
          min_best[i] = min_linear_100;
        }
    }

    best_d[i][NODE]   = d_best[NODE];
    best_d[i][NODE+1] = d_best[NODE+1];
    
    d[NODE]   = d_best[NODE];
    d[NODE+1] = d_best[NODE+1];

    d_av[NODE] = (d[NODE]+d_copy[NODE+1])/2;
    y[NODE] = y[NODE] + rho*(d[NODE]-d_av[NODE]);
    d_copy[NODE] = d[NODE];
    // Send to other arduinos!

  }
  while(1);
  
}





















