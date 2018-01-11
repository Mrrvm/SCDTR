%The system
k11 = 0.445; k12 = 0.033; k21 = 0.021; k22 = 0.333;
L1 = 50; o1 = 0; L2 = 50; o2 = 0;
%k11 = 2; k12 = 1; k21 = 1; k22 = 2;
%L1 = 300; o1 = 30; L2 = 300; o2 = 0;
K = [k11, k12 ; k21 , k22];
L = [L1;L2]; o = [o1;o2];

%The cost function
c1 = 1; c2 = 1; q1 = 0; q2 = 0;
c = [c1 c2]; Q = [q1 0; 0 q2];

% SOLVE WITH CONSENSUS
rho = 0.01;
%node 1 initialization
d1 = [0;0];
d1_av = [0;0];
d2_copy = [0;0];
y1 = zeros(2,50);%[0;0];
y1(:,1)=[4.36;1.24];
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
   z11 = -c1 - y1(1,i) + rho*d1_av(1);
   z12 = -y1(2,i) + rho*d1_av(2);
   u1 = o1-L1;
   u2 = 0;
   u3 = 255;
   p11 = 1/(rho+q1);
   p12 = 1/rho;
   n = k11*k11*p11 + k12*k12*p12;
   w1 = -k11*p11*z11-k12*z12*p12;
   w2 = -z11*p11;
   w3 = z11*p11;
   %compute unconstrained minimum
   d11u(i) = p11*z11;
   d12u(i) = p12*z12;
   %check feasibility of unconstrained minimum using local constraints
   if (d11u(i) < 0), sol_unconstrained = 0; end;
   if (d11u(i) > 255), sol_unconstrained = 0; end;
   if (k11*d11u(i) + k12*d12u(i) < L1-o1), sol_unconstrained = 0; end;
   % compute function value and if best store new optimum
   if sol_unconstrained, 
        min_unconstrained = 0.5*q1*d11u(i)^2 + c1*d11u(i) + y1(1,i)*(d11u(i)-d1_av(1)) + ...
           y1(2,i)*(d12u(i)-d1_av(2)) + rho/2*(d11u(i)-d1_av(1))^2 + rho/2*(d12u(i)-d1_av(2))^2;
       if min_unconstrained < min_best_1(i),
           d11_best = d11u(i);
           d12_best = d12u(i);
           min_best_1(i) = min_unconstrained;
           i1(i)=0;
       end;
   end;
   %compute minimum constrained to linear boundary   
   d11bl(i) = p11*z11+p11*k11/n*(w1-u1);
   d12bl(i) = p12*z12+p12*k12/n*(w1-u1);

   %u1
   %p11*z11
   %p12*z12
   %p11*k11
   %p12*k12
   %1/n
   
   
   %check feasibility of minimum constrained to linear boundary
   if (d11bl(i) < 0), sol_boundary_linear = 0; end;
   if (d11bl(i) > 255), sol_boundary_linear = 0; end;
   % compute function value and if best store new optimum
   if sol_boundary_linear, 
        min_boundary_linear = 0.5*q1*d11bl(i)^2 + c1*d11bl(i) + y1(1,i)*(d11bl(i)-d1_av(1)) + ...
           y1(2,i)*(d12bl(i)-d1_av(2)) + rho/2*(d11bl(i)-d1_av(1))^2 + rho/2*(d12bl(i)-d1_av(2))^2;
       if min_boundary_linear < min_best_1(i),
           d11_best = d11bl(i);
           d12_best = d12bl(i);
           min_best_1(i) = min_boundary_linear;
           i1(i)=1;
       end;
   end;
   %compute minimum constrained to 0 boundary
   d11b0(i) = 0;
   d12b0(i) = p12*z12;
   %check feasibility of minimum constrained to 0 boundary
   if (d11b0(i) > 255), sol_boundary_0 = 0; end;
   if (k11*d11b0(i) + k12*d12b0(i) < L1-o1), sol_boundary_0 = 0; end;
   % compute function value and if best store new optimum
   if sol_boundary_0, 
        min_boundary_0 = 0.5*q1*d11b0(i)^2 + c1*d11b0(i) + y1(1,i)*(d11b0(i)-d1_av(1)) + ...
           y1(2,i)*(d12b0(i)-d1_av(2)) + rho/2*(d11b0(i)-d1_av(1))^2 + rho/2*(d12b0(i)-d1_av(2))^2;
       if min_boundary_0 < min_best_1(i),
           d11_best = d11b0(i);
           d12_best = d12b0(i);
           min_best_1(i) = min_boundary_0;
           i1(i)=2;
       end;
   end;
   %compute minimum constrained to 100 boundary
   d11b100(i) = 255;
   d12b100(i) = p12*z12;
   %check feasibility of minimum constrained to 100 boundary
   if (d11b100(i) < 0), sol_boundary_100 = 0; end;
   if (k11*d11b100(i) + k12*d12b100(i) < L1-o1), sol_boundary_100 = 0; end;
   % compute function value and if best store new optimum
   if sol_boundary_100, 
        min_boundary_100 = 0.5*q1*d11b100(i)^2 + c1*d11b100(i) + y1(1,i)*(d11b100(i)-d1_av(1)) + ...
           y1(2,i)*(d12b100(i)-d1_av(2)) + rho/2*(d11b100(i)-d1_av(1))^2 + rho/2*(d12b100(i)-d1_av(2))^2;
       if min_boundary_100 < min_best_1(i),
           d11_best = d11b100(i);
           d12_best = d12b100(i);
           min_best_1(i) = min_boundary_100;
           i1(i)=3;
       end;
   end;
   % compute minimum constrained to linear and zero boundary
   common = (rho+q1)/((rho+q1)*n-k11*k11);
   det1 = common;
   det2 = -k11*common;
   det3 = det2;
   det4 = n*(rho+q1)*common;
   x1 = det1*w1 + det2*w2;
   x2 = det3*w1 + det4*w2;
   v1 = det1*u1 + det2*u2; %u2 = 0 so this can be simplified
   v2 = det3*u1 + det4*u2; %u2 = 0 so this can be simplified
   d11l0(i) = p11*z11+p11*k11*(x1-v1)+p11*(x2-v2);
   d12l0(i) = p12*z12+p12*k12*(x1-v1);
   %check feasibility
   if (d11l0(i) > 255), sol_linear_0 = 0; end;
   % compute function value and if best store new optimum
   if sol_linear_0, 
        min_linear_0 = 0.5*q1*d11l0(i)^2 + c1*d11l0(i) + y1(1,i)*(d11l0(i)-d1_av(1)) + ...
           y1(2,i)*(d12l0(i)-d1_av(2)) + rho/2*(d11l0(i)-d1_av(1))^2 + rho/2*(d12l0(i)-d1_av(2))^2;
       if min_linear_0 < min_best_1(i),
           d11_best = d11l0(i);
           d12_best = d12l0(i);
           min_best_1(i) = min_linear_0;
           i1(i)=4;
       end;
   end;
   % compute minimum constrained to linear and 100 boundary
   common = (rho+q1)/((rho+q1)*n-k11*k11);
   det1 = common;
   det2 = k11*common;
   det3 = det2;
   det4 = n*(rho+q1)*common;
   x1 = det1*w1 + det2*w3;
   x2 = det3*w1 + det4*w3;
   v1 = det1*u1 + det2*u3; 
   v2 = det3*u1 + det4*u3; 
   d11l100(i) = p11*z11+p11*k11*(x1-v1)-p11*(x2-v2);
   d12l100(i) = p12*z12+p12*k12*(x1-v1);
   %check feasibility
   if (d11l100(i) < 0), sol_linear_100 = 0; end;
   % compute function value and if best store new optimum
   if sol_linear_100, 
        min_linear_100 = 0.5*q1*d11l100(i)^2 + c1*d11l100(i) + y1(1,i)*(d11l100(i)-d1_av(1)) + ...
           y1(2,i)*(d12l100(i)-d1_av(2)) + rho/2*(d11l100(i)-d1_av(1))^2 + rho/2*(d12l100(i)-d1_av(2))^2;
       if min_linear_100 < min_best_1(i),
           d11_best = d11u(i);
           d12_best = d12u(i);
           min_best_1(i) = min_linear_100;
           i1(i)=5;
       end;
   end;
   %store data and save for next cycle
   best_d11(i) = d11_best;
   best_d12(i) = d12_best;
   d1 = [d11_best;d12_best];
   %DEBUG: check with matlab quadprog
   Q = [q1+rho, 0; 0 rho];
   c = [c1+y1(1,i)-rho*d1_av(1),y1(2,i)-rho*d1_av(2)];
   A = [-k11 -k12; -1 0; 1 0];
   b = [o1-L1, 0, 255];
   d1_ = quadprog(Q,c,A,b,[],[],[],[]);
   %
   
   % send node 1 solution to neighboors
   d1_copy = d1;
   
   % node 2 
   d21_best = -1;
   d22_best = -1;
   min_best_2(i) = 100000; %big number
   sol_unconstrained = 1;
   sol_boundary_linear = 1;
   sol_boundary_0 = 1;
   sol_boundary_100 = 1;
   sol_linear_0 = 1;
   sol_linear_100 = 1;
   z22 = -c2 - y2(2) + rho*d2_av(2);
   z21 = -y2(1) + rho*d2_av(1);
   u1 = o2-L2;
   u2 = 0;
   u3 = 255;
   p22 = 1/(rho+q2);
   p21 = 1/rho;
   n = k22*k22*p22 + k21*k21*p21;
   w1 = -k22*p22*z22-k21*z21*p21;
   w2 = -z22*p22;
   w3 = z22*p22;
   %compute unconstrained minimum
   d21u(i) = p21*z21;
   d22u(i) = p22*z22;
   %check feasibility of unconstrained minimum using local constraints
   if (d22u(i) < 0), sol_unconstrained = 0; end;
   if (d22u(i) > 255), sol_unconstrained = 0; end;
   if (k21*d21u(i) + k22*d22u(i) < L2-o2), sol_unconstrained = 0; end;
   % compute function value and if best store new optimum
   if sol_unconstrained, 
        min_unconstrained = 0.5*q2*d22u(i)^2 + c2*d22u(i) + y2(1)*(d21u(i)-d2_av(1)) + ...
           y2(2)*(d22u(i)-d2_av(2)) + rho/2*(d21u(i)-d2_av(1))^2 + rho/2*(d22u(i)-d2_av(2))^2;
       if min_unconstrained < min_best_2(i),
           d21_best = d21u(i);
           d22_best = d22u(i);
           min_best_2(i) = min_unconstrained;
           i2(i)=0;
       end;
   end;
   %compute minimum constrained to linear boundary   
   d21bl(i) = p21*z21+p21*k21/n*(w1-u1);
   d22bl(i) = p22*z22+p22*k22/n*(w1-u1);
   %check feasibility of minimum constrained to linear boundary
   if (d22bl(i) < 0), sol_boundary_linear = 0; end;
   if (d22bl(i) > 255), sol_boundary_linear = 0; end;
   % compute function value and if best store new optimum
   if sol_boundary_linear, 
        min_boundary_linear = 0.5*q2*d22bl(i)^2 + c2*d22bl(i) + y2(1)*(d21bl(i)-d2_av(1)) + ...
           y2(2)*(d22bl(i)-d2_av(2)) + rho/2*(d21bl(i)-d2_av(1))^2 + rho/2*(d22bl(i)-d2_av(2))^2;
       if min_boundary_linear < min_best_2(i),
           d21_best = d21bl(i);
           d22_best = d22bl(i);
           min_best_2(i) = min_boundary_linear;
           i2(i)=1;     
       end;
   end;
   %compute minimum constrained to 0 boundary
   d22b0(i) = 0;
   d21b0(i) = p21*z21;
   %check feasibility of minimum constrained to 0 boundary
   if (k21*d21b0(i) + k22*d22b0(i) < L2-o2), sol_boundary_0 = 0; end;
   if (d22b0(i) > 255), sol_boundary_0 = 0; end;
   % compute function value and if best store new optimum
   if sol_boundary_0, 
        min_boundary_0 = 0.5*q2*d22b0(i)^2 + c2*d22b0(i) + y2(1)*(d21b0(i)-d2_av(1)) + ...
           y2(2)*(d22b0(i)-d2_av(2)) + rho/2*(d21b0(i)-d2_av(1))^2 + rho/2*(d22b0(i)-d2_av(2))^2;
       if min_boundary_0 < min_best_2(i),
           d21_best = d21b0(i);
           d22_best = d22b0(i);
           min_best_2(i) = min_boundary_0;
           i2(i)=2;
       end;
   end;
   %compute minimum constrained to 100 boundary
   d22b100(i) = 255;
   d21b100(i) = p21*z21;
   %check feasibility of minimum constrained to 100 boundary
   if (k21*d21b100(i) + k22*d22b100(i) < L2-o2), sol_boundary_100 = 0; end;
   if (d22b100(i) < 0), sol_boundary_100 = 0; end;
   % compute function value and if best store new optimum
   if sol_boundary_100, 
        min_boundary_100 = 0.5*q2*d22b100(i)^2 + c2*d22b100(i) + y2(1)*(d21b100(i)-d2_av(1)) + ...
           y2(2)*(d22b100(i)-d2_av(2)) + rho/2*(d21b100(i)-d2_av(1))^2 + rho/2*(d22b100(i)-d2_av(2))^2;
       if min_boundary_100 < min_best_2(i),
           d21_best = d21b100(i);
           d22_best = d22b100(i);
           min_best_2(i) = min_boundary_100;
           i2(i)=3;
       end;
   end;
   % compute minimum constrained to linear and zero boundary
   common = (rho+q2)/((rho+q2)*n-k22*k22);
   det1 = common;
   det2 = -k22*common;
   det3 = det2;
   det4 = n*(rho+q2)*common;
   x1 = det1*w1 + det2*w2;
   x2 = det3*w1 + det4*w2;
   v1 = det1*u1 + det2*u2; %u2 = 0 so this can be simplified
   v2 = det3*u1 + det4*u2; %u2 = 0 so this can be simplified
   d22l0(i) = p22*z22+p22*k22*(x1-v1)+p22*(x2-v2);
   d21l0(i) = p21*z21+p21*k21*(x1-v1);
   %check feasibility
   if(d22l0(i) > 255), sol_linear_0 = 0; end;
   % compute function value and if best store new optimum
   if sol_linear_0, 
        min_linear_0 = 0.5*q2*d22l0(i)^2 + c2*d22l0(i) + y2(1)*(d21l0(i)-d2_av(1)) + ...
           y2(2)*(d22l0(i)-d2_av(2)) + rho/2*(d21l0(i)-d2_av(1))^2 + rho/2*(d22l0(i)-d2_av(2))^2;
       if min_linear_0 < min_best_2(i),
           d21_best = d21l0(i);
           d22_best = d22l0(i);
           min_best_2(i) = min_linear_0;
           i2(i)=4;
       end;
   end;
   % compute minimum constrained to linear and 100 boundary
   common = (rho+q2)/((rho+q2)*n-k22*k22);
   det1 = common;
   det2 = k22*common;
   det3 = det2;
   det4 = n*(rho+q2)*common;
   x1 = det1*w1 + det2*w3;
   x2 = det3*w1 + det4*w3;
   v1 = det1*u1 + det2*u3; 
   v2 = det3*u1 + det4*u3; 
   d22l100(i) = p22*z22+p22*k22*(x1-v1)-p22*(x2-v2);
   d21l100(i) = p21*z21+p21*k21*(x1-v1);
   %check feasibility
   if (d22l100(i) < 0), sol_linear_100 = 0; end;
   %now must choose the minimum among the feasible solutions
   % compute function value and if best store new optimum
   if sol_linear_100, 
        min_linear_100 = 0.5*q2*d22l100(i)^2 + c2*d22l100(i) + y2(1)*(d21l100(i)-d2_av(1)) + ...
           y2(2)*(d22l100(i)-d2_av(2)) + rho/2*(d21l100(i)-d2_av(1))^2 + rho/2*(d22l100(i)-d2_av(2))^2;
       if min_linear_100 < min_best_2(i),
           d21_best = d21u(i);
           d22_best = d22u(i);
           min_best_2(i) = min_linear_100;
           i2(i)=5;
       end;
   end;
   %store data and save for next cycle
   best_d21(i) = d21_best;
   best_d22(i) = d22_best;
   d2 = [d21_best;d22_best];
   %DEBUG: check with matlab quadprog
   Q = [rho, 0; 0 rho+q2];
   c = [y2(1)-rho*d2_av(1),c2+y2(2)-rho*d2_av(2)];
   A = [-k21 -k22;0 -1; 0 1];
   b = [o2-L2, 0, 255];
   d2_ = quadprog(Q,c,A,b,[],[],[],[]);
   
   % Compute average with available data
   d2_av = (d1_copy+d2)/2;
   % Update local lagrangian
   y2 = y2 + rho*(d2-d2_av);
   % send solution to neighbors
   d2_copy = d2;
   
   %compute average with available knowledge
   d1_av = (d1+d2_copy)/2;
   %update local lagrangian
   %y1(:,i)
   %d1
   %d1_av
   y1(:,i) = y1(:,i) + rho*(d1-d1_av);
   %y1(:,i)
   
   %save data for plots
   av1(i) = d1_av(1);
   av2(i) = d1_av(2);
end;

% SOLVE WITH MATLAB QUADPROG
%The cost function
c1 = 4; c2 = 1; q1 = 0; q2 = 0;
c = [c1 c2]; Q = [q1+rho 0; 0 q2];
A = -K; b = [o1-L1; o2-L2];
lb = [0;0]; ub = [255;255];
disp('Matlab solutions')
d = quadprog(Q,c,A,b,[],[],lb,ub)
l = K*d+o

disp('Consensus Solutions')
d_ = d2_av
l_ = K*d_+o
%Plots
figure(10);
plot(1:50, av1, 1:50, av2);
legend('d_1','d_2');
title('Evolucao de d_i');
ylabel('d_i');
xlabel('iteracao');
figure(15);
t = 0:255;
constr1 = (L1-o1)/0.033-(0.445/0.033)*t;
constr2 = (L2-o2)/0.333-(0.012/0.333)*t;
%constr1 = (L1-o1)/1-(2/1)*t;
%constr2 = (L2-o2)/2-(1/2)*t;
[x,y]=meshgrid(t,t);
hold on;
z = c1*x+c2*y+q1*x.^2+q2*y.^2;
contour(x,y,z);
plot(t,constr1,t,constr2,'LineWidth',2);
plot(t,constr1,'LineWidth',2);
plot(t,zeros(size(t)),'k','LineWidth',2);
plot(zeros(size(t)),t,'k','LineWidth',2);
plot(t,255*ones(size(t)),'k','LineWidth',2);
plot(255*ones(size(t)),t,'k','LineWidth',2);
plot(av1,av2,'--','LineWidth',2);
plot(av1,av2,'bo');
title('Trajetoria');
xlabel('d_1');
ylabel('d_2');
plot(d(1),d(2),'r*')
axis([-10,260,-10,260]);
hold off;

    

