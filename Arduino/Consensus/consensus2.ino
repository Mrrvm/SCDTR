#define ITER_N 50
#define NODE 0 //get this from eeprom (matlab starts at 1...)
#define N_NODES 2 

double get_min(double * d_av, double * d, double * c, double * y, double rho, double * q) {
  double min;
  double diff =  d[NODE]-d_av[NODE];
  double diff1 = d[NODE+1]-d_av[NODE+1];
      
  min = 0.5*q[NODE]*d[NODE]*d[NODE] 
          + c[NODE]*d[NODE] 
          + y[NODE]*diff 
          + y[NODE+1]*diff1 
          + (rho/2)*diff*diff 
          + (rho/2)*diff1*diff1;
  return min;
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  // The system - this args come from outside
  double k_self = 2, k1 = 1; 
  double k[N_NODES] = {k_self, k1};
  double L = 150; 
  double o = 30; 
  
  // The cost function - this args come from outside
  double c_self = 1; 
  double q_self = 0.0;
  double c[N_NODES] = {c_self, 0}; 
  double q[N_NODES] = {q_self, 0};
  
  // Initialization 
  double rho                  = 0.01;
  double d[N_NODES]           = {0};
  double d_av[N_NODES]        = {0};
  double d_copy[N_NODES]      = {0};
  double y[N_NODES]           = {0};
  double min_best[ITER_N]     = {0};
  double z[N_NODES]           = {0};
  double p[N_NODES]           = {0};
  double d_best[N_NODES]      = {0};
  double du[N_NODES]          = {0};
  double dbl[N_NODES]         = {0};
  double db0[N_NODES]         = {0};
  double db100[N_NODES]       = {0};
  double dl100[N_NODES]       = {0};
  double dl0[N_NODES]         = {0};
  double best_d[ITER_N][NODE];
  // this the solution computed by other arduinos
  // d_other[arduino #][solution #]
  double d_others[N_NODES][N_NODES];
  d_others[NODE+1][NODE] = 0;
  
  bool sol_unconstrained;
  bool sol_boundary_linear;
  bool sol_boundary_0;
  bool sol_boundary_100;
  bool sol_linear_0;
  bool sol_linear_100;
  double n;
  double min_unconstrained;
  double min_boundary_linear;
  double min_boundary_0;
  double min_boundary_100;
  double min_linear_100;
  double min_linear_0;
  double w1, w2, w3; 
  double det1, det2, det3, det4;
  double x1, x2;
  double v1, v2;
  double common;
  //this will have to be with for loops
  // cuz there will be more boudaries if N_NODES is bigger...
  double u1, u2, u3;
  
  for(int i=0; i<ITER_N; i++) {
    
    d_best[NODE] = -1;
    d_best[NODE+1] = -1;
    min_best[i] = 100000;
    sol_unconstrained = 1;
    sol_boundary_linear = 1;
    sol_boundary_0 = 1;
    sol_boundary_100 = 1;
    sol_linear_0 = 1;
    sol_linear_100 = 1;
    
    z[NODE]   = -c[NODE]   - y[NODE]   + rho*d_av[NODE];
    z[NODE+1] = -y[NODE+1] + rho*d_av[NODE+1];
    
    u1 = o - L;
    u2 = 0;
    u3 = 100;

    p[NODE]   = 1/(rho+q[1]);
    p[NODE+1] = 1/rho;
  
    // this must be a for loop
    n  =  k[NODE]*k[NODE]*p[NODE] + k[NODE+1]*k[NODE+1]*p[NODE+1];
    w1 = -k[NODE]*z[NODE]*p[NODE] - k[NODE+1]*z[NODE+1]*p[NODE+1];
    w2 = -z[NODE]*p[NODE];
    w3 = -(w2);
    
    // Unconstrained
    du[NODE]   =  z[NODE]*p[NODE];
    du[NODE+1] =  z[NODE+1]*p[NODE+1];
    
    if(du[NODE] < 0) { sol_unconstrained = 0;}
    if(du[NODE] > 100) { sol_unconstrained = 0;}
    if(k[NODE]*du[NODE] + k[NODE+1]*du[NODE+1] < L-o) { sol_unconstrained = 0;}
    if(sol_unconstrained) {
        min_unconstrained = get_min((double *) d_av, (double *) du, (double *) c, (double *) y, rho, (double *) q);
        if (min_unconstrained < min_best[i]) {
          d_best[NODE]   = du[NODE];
          d_best[NODE+1] = du[NODE+1];
          min_best[i] = min_unconstrained;
        }
    }

    // Boundary Linear
    dbl[NODE] = p[NODE]*z[NODE] + (p[NODE]*k[NODE]/n)*(w1-u1);
    dbl[NODE+1] = p[NODE+1]*z[NODE+1] + (p[NODE+1]*k[NODE+1]/n)*(w1-u1);
    if(dbl[NODE] < 0) {sol_boundary_linear = 0;}
    if(dbl[NODE] > 100) {sol_boundary_linear = 0;}
    if(sol_boundary_linear) {
        min_boundary_linear = get_min((double *) d_av, (double *) dbl, (double *) c, (double *) y, rho, (double *) q);
        if (min_boundary_linear < min_best[i]) {
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
        min_boundary_0 = get_min((double *) d_av, (double *) db0, (double *) c, (double *) y, rho, (double *) q);
        if (min_boundary_0 < min_best[i]) {
          d_best[NODE]   = db0[NODE];
          d_best[NODE+1] = db0[NODE+1];
          min_best[i] = min_boundary_0;
        }
    }

    // Boundary 100
    db100[NODE] = 100;
    db100[NODE+1] = p[NODE+1]*z[NODE+1];
    if(db100[NODE] < 0) {sol_boundary_100 = 0;}
    if(k[NODE]*db100[NODE] + k[NODE+1]*db100[NODE+1] < L-o) {sol_boundary_100 = 0;}
    if(sol_boundary_100) {
        min_boundary_100 = get_min((double *) d_av, (double *) db100, (double *) c, (double *) y, rho, (double *) q);
        if (min_boundary_100 < min_best[i]) {
          d_best[NODE]   = db100[NODE];
          d_best[NODE+1] = db100[NODE+1];
          min_best[i] = min_boundary_100;
        }
    }

    common = (rho+q[NODE])/((rho+q[NODE])*n-k[NODE]*k[NODE]);
    det1 = common;
    det2 = -k[NODE]*common;
    det3 = det2;
    det4 = n*(rho+q[NODE])*common;
    x1 = det1*w1 + det2*w2;
    x2 = det3*w1 + det4*w2;
    v1 = det1*u1 + det4*u2;
    v2 = det3*u1 + det4*u2;
    // Linear 0
    dl0[NODE]   = p[NODE]*z[NODE] + p[NODE]*k[NODE]*(x1-v1)+p[NODE]*(x2-v2);
    dl0[NODE+1] = p[NODE+1]*z[NODE+1] + p[NODE+1]*k[NODE+1]*(x1-v1);
    if(dl0[NODE] > 100) {sol_linear_0 = 0;}
    if(sol_linear_0) {
        min_linear_0 = get_min((double *) d_av, (double *) dl0, (double *) c, (double *) y, rho, (double *) q);
        if (min_linear_0 < min_best[i]) {
          d_best[NODE]   = dl0[NODE];
          d_best[NODE+1] = dl0[NODE+1];
          min_best[i] = min_linear_0;
        }
    }

    common = (rho+q[NODE])/((rho+q[NODE])*n-k[NODE]*k[NODE]);
    det1 = common;
    det2 = k[NODE]*common;
    det3 = det2;
    det4 = n*(rho+q[NODE])*common;
    x1 = det1*w1 + det2*w3;
    x2 = det3*w1 + det4*w3;
    v1 = det1*u1 + det2*u3;
    v2 = det3*u1 + det4*u3;

    // Linear 100
    dl100[NODE]   = p[NODE]*z[NODE] + p[NODE]*k[NODE]*(x1-v1)-p[NODE]*(x2-v2);
    dl100[NODE+1] = p[NODE+1]*z[NODE+1] + p[NODE+1]*k[NODE+1]*(x1-v1);
    if(dl100[NODE] < 0) {sol_linear_100 = 0;}
    if(sol_linear_100) {
        min_linear_100 = get_min((double *) d_av, (double *) dl100, (double *) c, (double *) y, rho, (double *) q);
        if (min_linear_100 < min_best[i]) {
          d_best[NODE]   = dl100[NODE];
          d_best[NODE+1] = dl100[NODE+1];
          min_best[i] = min_linear_100;
        }
    }

    best_d[i][NODE]   = d_best[NODE];
    best_d[i][NODE+1] = d_best[NODE+1];
    
    d[NODE]   = d_best[NODE];
    d[NODE+1] = d_best[NODE+1];

    d_av[NODE]   = (d[NODE]   + d_others[NODE+1][NODE])/N_NODES;
    d_av[NODE+1] = (d[NODE+1] + d_others[NODE+1][NODE+1])/N_NODES;
    y[NODE]   = y[NODE]   + rho*(d[NODE]-d_av[NODE]);
    y[NODE+1] = y[NODE+1] + rho*(d[NODE+1]-d_av[NODE+1]);
    d_copy[NODE] = d[NODE];
    d_copy[NODE+1] = d[NODE+1];
    // Send to other arduinos!

    Serial.println("result is "+String(i));
    Serial.println(d[NODE]);
    Serial.println(d[NODE+1]);

  }
  while(1);
  
}





















