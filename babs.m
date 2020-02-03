1;
pkg load control;
a = -0.0060949;
A = [0 1 0 0;0 -0.10572 1892.704 0; 0 0 0 1; 0 a 193.8901 0];
B = [0; 0.8997; 0; 0.05186];
Q = eye(4);
R = .1;
K = lqr(A,B,Q,R);
y0 = [5;0;0;0];
y = [2.5;0;0;0];
U = -K*(y-y0); 
cost = 0;
for i = 1:1000
   cost = cost +  (y'*Q*y +U'*R*U);
   y_new = A*y +B*U;
   y = y_new;
   U = -K*(y-y0);
endfor
