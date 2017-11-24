%The system
k11 = 2; k12 = 1; k21 = 1; k22 = 2;
L1 = 150; o1 = 30; L2 = 80; o2 = 0;
K = [k11, k12 ; k21 , k22];
L = [L1;L2]; o = [o1;o2];

%The cost function
c1 = 1; c2 = 0; q1 = 0.0; q2 = 0.0;
c = [c1 c2]; Q = [q1 0; 0 q2];

% SOLVE WITH CONSENSUS
rho = 0.01;
%node 1 initialization
d1 = [0;0];
d1_av = [0;0];
d2_copy = [0;0];
y1 = [0;0];
k1 = [k11;k12]; 
%node 2 initialization
d2 = [0;0];
d2_av = [0;0];
d1_copy = [0;0];
y2 = [0;0];
k2 = [k21;k22]; 
%iterations
for i=1:50,
   % node 1
   d11_best = -1;
   d12_best = -1;
   min_best_1(i) = 100000; %big number
   sol_unconstrained = 1;
   sol_boundary_linear = 1;
   sol_boundary_0 = 1;
   sol_boundary_100 = 1;
   sol_linear_0 = 1;
   sol_linear_100 = 1;
   z11 = -c1 - y1(1) + rho*d1_av(1);
   z12 = -y1(2) + rho*d1_av(2);
end

