clear all global;
clc;

% Initializing State variables 
A = [0 1 0 0 0 0 ; ...
    0 -0.0104 0 0 0 0;... 
    0 0 0 1 0 0; ...
    0 0 0 -0.0104 0 0;...
    0 0 0 0 0 1;...
    0 0 0 0 0 -0.0208];
B = [0 0 0 0;... 
    -0.04167 0 0.04167 0;...
    0 0 0 0;... 
    0 -0.04167 0 0.04167;...
    0 0 0 0;...
    0.4 0.4 0.4 0.4];
C = [1 0 0 0 0 0;...
    0 0 1 0 0 0;...
    0 0 0 0 1 0];
D = [0 0 0 0;...
    0 0 0 0;...
    0 0 0 0];...
x0 = [0 0 0 0 0 0]';    % initial state of quadrotor is origin i.e 0,0,0
t = 0:1:529;             % defining a time matrix for plot of lsim
t1 = t';
q = [0 0 0 0];
r = ones(529,4);
u = [q;r]; %simulating step input with u=0 at t=0 and u=1 after that  

%Open loop system
s0 = ss(A,B,C,D);       %open loop state space system
figure(1); grid on;
step(s0);               %Step input - open loop w/o controller
figure(2);grid on;
lsim(s0,u,t,x0);        %simulated step response of the system


%Closed loop
p1 = -53 + 105i;         % arbitrary pole placement 
p2 = -53 - 105i;
p3 = -55 + 30i;
p4 = -55 - 30i;
p5 = -23.15;
p6 = -45;
p7 = [p1 p2 p3 p4 p5 p6];%consolidate poles into one
K1 = place(A,B,p7);      %places the poles for closed loop with controller
F = 31.6227*1.62*31.6227*1.62;        %reference input scalar
s1 = ss(A-B*K1,B*F,C,D); %closed loop system with reference input 

%plotting closed loop system response 
figure(3); grid on;
step(s1);                % step response 
figure(4);grid on;
lsim(s1,u,t,x0);       %simulated step response of closed loop system with controller 
%lsd = lsiminfo(lsim(s1,u,t,x0),t);


%Closed loop system with controller and observer used to plot 
%only the error  

pl7 = [p1 p2 p3 p4 p5 p6]*10;
L = place(A',C',pl7)';    %Placing poles 10 times farther than the closed loop poles
s2 = ss(A-L*C,B*F,C,D);   %Create a new state space system with to calculate error only 
figure(6);grid on;        
lsim(s2,u,t,[ 1 0 1 0 1 0]);          %Plots the error in observing x,y,z state variables 
figure(7);grid on;
step(s2)

ls1 = lsim(s1,u,t,x0);    %closed loop system with controller 
ls2 = lsim(s2,u,t,[1 0 1 0 1 0]);    %extrapolate error values e(x- xobs)
Xhat = ls1-ls2;           %Calculate estimated(observed)values
tt = 0:1:529;             %define a time matrix for plot of lsim

figure(88);grid on;        %compare fig 7 and figure 4   
subplot(3,1,1); grid on; 
plot(tt,Xhat(:,1));       %Plots the estimated value x,y,z state variables 
subplot(3,1,2); grid on;
plot(tt,Xhat(:,2));
subplot(3,1,3); grid on; 
plot(tt,Xhat(:,3));
legend("Observed Value");
%}

% defininig the whole state system with feedback with observer and
% controller as a whole 
% define new matrices for the system 
Z  = zeros(6);
AA = [(A-B*K1),(B*K1);... % this system will twice(12 state) the states in 
         Z , (A-L*C)];    % og sysytem 
BB = [B*F;...             % New B Matrix 
      0 0 0 0;...
      0 0 0 0;...
      0 0 0 0;...
      0 0 0 0;...
      0 0 0 0;...
      0 0 0 0];
CC = [1 0 0 0 0 0 0 0 0 0 0 0;... % New C matrix measuring the error in x,y,z 
      0 0 1 0 0 0 0 0 0 0 0 0;...
      0 0 0 0 1 0 0 0 0 0 0 0;...
      0 0 0 0 0 0 1 0 0 0 0 0;...
      0 0 0 0 0 0 0 0 1 0 0 0;...
      0 0 0 0 0 0 0 0 0 0 1 0];
DD = zeros(6,4);
s3 = ss(AA,BB,CC,DD);              % New state space system 
x00 = [ 0 0 0 0 0 0 1 0 1 0 1 0]'; % defining the new initial condition 
figure(9); grid on;
lsim(s3,u,t,x00);                  % plotting the actual x,y,z and their 
                                   % corresponding error values 
ls3 = lsim(s3,u,t,x00);

%Closed loop system with LQR
rho = 0.01;                        %Declaring various Q and R matrices for
Q1 = eye(6);                       %for comparision
R1 = rho*eye(4);
p = 1/((10)^2);                    %LQR penalty variable - 10% error therefore 10 cm
Q2 = [5 0 0 0 0 0;
      0 0.1 0 0 0 0;
      0 0 5 0 0 0;
      0 0 0 0.1 0 0;
      0 0 0 0 5 0;
      0 0 0 0 0 0.1];
R2 = rho*eye(4);
Q3 = C'*C;
R3 = rho*eye(4);
xd = [1 0 1 0 1 0]';
N = zeros(6,4);
[Kl_1,S1,e1] = lqr(s0,Q1,R1,N);   %extracting New gain Matrices with varying Q,R
[Kl_2,S2,e2] = lqr(s0,Q2,R2,N);
[Kl_3,S3,e3] = lqr(s0,Q3,R3,N);
sl_1 = ss(A-B*Kl_1, B*Kl_1*xd, C,D*Kl_1*xd);    %declaring various LAR system to compare the response
sl_2 = ss(A-B*Kl_2, B*Kl_2*xd, C,D*Kl_2*xd);
sl_3 = ss(A-B*Kl_3, B*Kl_3*xd, C,D*Kl_3*xd);
figure(10);grid on;
step(sl_1,sl_2,sl_3);             %Step response of closed loop system with LQR
legend("M1","M2","M3")
%[ls4_1,T1] = step(sl_1);
[ls4_2,T2] = step(sl_2);
%[ls4_3,T3] = step(sl_3);

figure(8);grid on;        %compare fig 7 and figure 4   
subplot(3,1,1); grid on; hold on
plot(tt(1:253),[Xhat(1:253,1) ls4_2(:,1)]);       %Plots the estimated value x,y,z state variables 
subplot(3,1,2); grid on;
plot(tt(1:253),[Xhat(1:253,2) ls4_2(:,2)]);
subplot(3,1,3); grid on; 
plot(tt(1:253),[Xhat(1:253,3) ls4_2(:,3)]);
legend("Measured Value","System With LQR");

figure(8);hold on;
y = 1;
subplot(3,1,1);hold on;
line([1,600],[y,y],'Color','black','LineStyle','--');
subplot(3,1,2);hold on;
line([1,600],[y,y],'Color','black','LineStyle','--');
subplot(3,1,3);hold on;
line([1,600],[y,y],'Color','black','LineStyle','--');


ql = [0];
rl = ones(1200,1);
ul = [ql;rl];                     %created a new input reference to simulate step response
t2 = 0:0.01:12;
figure(11);grid on;
lsim(sl_2,ul,t2,x0);              %simulated step response (similar to step response as shown in figure(9)
figure(9);grid on;