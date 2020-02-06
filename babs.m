1;
close all;
clear all;
clc;
pkg load control;

km= 2833.33;
ke= 0.0127;
Mp= 830;
Ip= 21632.83;
l= 3.025;
r= 3.25;
Res= 4;
Mw= 46;
Iw= 485.875;
g= 981;
beeta= (2*Mw + (2*Iw/(r**2)) + Mp);
alphaa= (Ip*beeta + 2*Mp*(l**2)*(Mw+ Iw/(r**2)));

A = [0 1 0 0;
     0 (2*km*ke*(Mp*l*r-Ip-Mp*(l**2)))/(Res*(r**2)*alphaa) ((Mp**2)*g*(l**2))/alphaa 0; 
     0 0 0 1; 
     0 (2*km*ke*(r*beeta-Mp*l))/(Res*(r**2)*alphaa) (Mp*g*l*beeta)/alphaa 0];
     
B = [0; 
     (2*km*(Ip+Mp*(l**2)-Mp*l*r))/(Res*r*alphaa);
     0;
     (2*km*(Mp*l-r*beeta))/(Res*r*alphaa)];

C = [1 0 0 0; 0 0 1 0];
D = [0;0]
Ts = 1/100;
sys_s = ss(A,B,C,D);
sys_d = c2d(sys_s,Ts,'zoh');

A_d = sys_d.A;
B_d = sys_d.B;
Q = [1000 0 0 0;
     0 10 0 0;
     0 0 100000 0;
     0 0 0 10];
R = 1500;
K = dlqr(A_d,B_d,Q,R);

Ac = [(A_d-B_d*K)];
Bc = [B_d];
Cc = [sys_d.C];
Dc = [sys_d.D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};
x_set = [1;0];
sys_cl = ss(Ac,Bc,Cc,Dc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);
t = 0:0.01:100;
ref = ((1/dcgain(sys_cl))*x_set)*ones(size(t));
[y,t,x]=lsim(sys_cl,ref,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');