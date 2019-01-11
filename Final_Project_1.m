clear all global;
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

s0 = ss(A,B,C,D);

K = [1 1 0 0 0 0;
    0 0 1 1 0 0;
    2 2 0 0 0 0;
    0 0 2 2 0 0;
    ];
A1 = A - B*K;
s1 = ss(A1,B,C,D);

p1 = -200 + 20i ; 
p2 = -200 - 20i ;
p3 = -0.01 ;
p4 = -0.04 ;
p5 = -0.03;
p6 = -0.02;
p7 = [p1 p2 p3 p4 p5 p6];

Ko = place(A,B,p7);

s2 = ss(A-B*Ko,B,C,D);
t = 0:0.01:700;
t1 = t';

figure(1);
step(s2); hold on; grid on;
st = stepinfo(s2);

%{
figure(2);
impulse(s2); grid on;
%}


 q = [0 0 0 0];
 r = ones(70000,4);
 u = [q;r];
 x0 = [0 0 0 0 0 0]'
 figure(2);
 lsim(s2,u,t,x0);
 grid on;
 %LQR
 Q = 10*eye(6);
R = eye(4);
N = zeros(6,4);
[Kl,S,e] = lqr(s0,Q,R,N);
Al = A - B*Kl;
sl = ss(Al,B,C,D);
figure(3);
step(sl);grid on;
figure(4);
lsim(sl,u,t,x0);
grid on;
 
 
while i>0
    n1 = randi([-1 1],1,1)
    p1 = randi([-10 0],1,1) + n1i;
    p2 = randi([-10 0],1,1) - n1i;
    
    for j = 1:70002
        if ls1(j,1)<=1.1 && ls1(j,1)>=0.9 && ls1(j,2)<=1.1 && ls1(j,2)>=0.9 && ls1(j,3)<=1.1 && ls1(j,3)>=0.9
            p1;
            p2;
            break;
        end
    end
end    
        
 
 
 %{
 L = place(A',C',p7)';
 
 s2l = ss(A-L*C,B,C,D);
 y1 = step(s2);
 y2 = step(s2l);
 figure(1);
 step(s2);
 hold on;
 grid on:
 step(s2l);
 legend('ko','L');
 ee = y1-y2;
 tt = 0:1:138;
 ee11 = ee(:,1,1);
 figure (2);
 plot(tt,ee11);
 grid on;

 lsim(s0,u,t,x0)
 %}


