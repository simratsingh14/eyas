1;
close all;
clear all;
clc;
pkg load control;

km= 0.27785508333;
ke= 0.38197186342;
Mp= 0.830;
Ip= 0.0020478;
l= 0.03025;
r= 0.0325;
Res= 4;
Mw= 0.046;
Iw= .0000485875;
g= 9.81;
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


C = eye(4);
D = [0;0;0;0];

    
%A = [0 1 0 0; 0 -0.0097 11.1594 0; 0 0 0 1; 0 -0.00293 172.1160 0];
%B = [0; 0.0815; 0; 0.2456];    
 Ts = 0.003;
sys_s = ss(A,B,C,D);
sys_d = c2d(sys_s,Ts,'zoh');
 
A_d = sys_d.A;
B_d = sys_d.B;

%Q = [8 0 0 0;
     %0 2 0 0;
     %0 0 16e5 0;
     %0 0 0 4];

%Q = [1e8 0 0 0;
     %0 8e3 0 0;
     %0 0 12 0;
     %0 0 0 1e5];
     
Q = [1600 0 0 0;
     0 400 0 0;
     0 0 364 0;
     0 0 0 400];     
R = 1;
K = dlqr(A_d,B_d,Q,R)
 

Ac = [(A_d-B_d*K)];
Bc = [B_d];
Cc = [sys_d.C];
Dc = [sys_d.D];


x_initial = [0,0,0.025,0];
x_set = [1;2;2;1];
sys_cl = ss(Ac,Bc,Cc,Dc,Ts);
t = 0:0.01:20;
 
 
[y,t,x] = initial(sys_cl,x_initial,t);
 
for i= 1:size(t)(1)
  u(i) = -K*x(i,:)';
endfor
 
figure;
subplot(3,2,1);
hold on;
plot(t,y(:,1));
title('Distance');
plot(t,0.025*ones(size(t)));
plot(t,-0.025*ones(size(t)));
hold off;
 
subplot(3,2,2);
plot(t,y(:,2));
title('velocity');
 
 
subplot(3,2,3);
plot(t,y(:,3));
title('angle');
 
 
subplot(3,2,4);
plot(t,y(:,4));
title('angular velocity');
 
subplot(3,2,5);
hold on;
plot(t,u');
plot(t,12*ones(size(t)));
plot(t,-12*ones(size(t)));
hold off;
title('effort');
pwm = u'*(255/12);
subplot(3,2,6);
plot(t,pwm);
title('PWM')