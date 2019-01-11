clear all global;
clc;
%State variables 
A = [0 1 0 0 0 0 ; 
    0 -0.0104 0 0 0 0; 
    0 0 0 1 0 0; 
    0 0 0 -0.0104 0 0;
    0 0 0 0 0 1;
    0 0 0 0 0 -0.0208];
B = [0 0 0 0; 
    -0.04167 0 0.04167 0;
    0 0 0 0; 
    0 -0.04167 0 0.04167;
    0 0 0 0;
    0.4 0.4 0.4 0.4];
C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];
D = [0 0 0 0;
    0 0 0 0;
    0 0 0 0];
x0 = [0 0 0 0 0 0]';
%lsim
t = 0:0.01:700;
t1 = t';
q = [0 0 0 0];
r = ones(70000,4);
u = [q;r];
%Open loop
s0 = ss(A,B,C,D);
%figure(1);
grid on;
%step(s0);
figure(2);
grid on;
lsim(s0,u,t,x0);
%Closed loop
p1 = -1 + 20i; 
p2 = -1 - 20i;
p3 = -1 + 20i;
p4 = -1 - 20i;
p5 = -0.04;
p6 = -0.02;
p7 = [p1 p2 p3 p4 p5 p6];
K1 = place(A,B,p7);
F = 1;
s1 = ss(A-B*K1,B*F,C,D);
%figure(3);
grid on;
%step(s1);
figure(4);grid on;
lsim(s1,u,t,x0);

%Observer
pl1 = -10 + 20i; 
pl2 = -10 - 20i;
pl3 = -10 + 20i;
pl4 = -10 - 20i;
pl5 = -0.04*0.99;
pl6 = -0.02*1;
pl7 = [pl1 pl2 pl3 pl4 pl5 pl6];
L = place(A',C',pl7)';
s2 = ss(A-L*C,B*F,C,D);
%figure(5);
grid on;
%step(s2);
figure(6);
grid on;
lsim(s2,u,t,x0);
ls2 = lsim(s2,u,t,x0);
y2 = ls2(:,1);
ls1 = lsim(s1,u,t,x0);
y1 = ls1(:,1);
ee = y1-y2;
tt = 0:1:70000;
figure(7);
plot(tt,ee);
grid on;
Z = zeros(6);
AA = [ (A-B*K1) (B*K1);
    Z (A-L*C)];
BB = [B*F;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0];
CC = [1 0 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0 0 0];
DD = zeros(4);
s4 = ss(AA,BB,CC,DD);
x00 = [ 0 0 0 0 0 0 0 0 0 0 0 0]';
figure(10); grid on;
lsim(s4,u,t,x00);
%{
LQR
Q = 10*eye(6);
R = eye(4);
N = zeros(6,4);
[K4,S,e] = lqr(s0,Q,R,N);
A4 = A - B*K4;
s3 = ss(A4,B,C,D);
figure(8);
grid on;
step(s3);
figure(9);
grid on;
lsim(s3,u,t,x0);
grid on;
%}
%StepInfo
st0 = stepinfo(s0); %Openloop
st1 = stepinfo(s1); %Closedloop
st2 = stepinfo(s2); %Observer
%st3 = stepinfo(s3); %LQR
