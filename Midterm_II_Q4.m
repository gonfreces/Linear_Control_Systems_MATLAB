% Midterm Paper Q4 - Aircraft lateral dynamics
% JSH 11/10/18

clear all global;

% Lateral Dynamics, M = 0.84, u0 = 825 ft/s, h = 33,000 ft
A = [-0.0869 0 0.039 -1; ...
     -4.424 -1.184 0 0.335; ...
     0 1 0 0; ...
     2.148 -0.021  0 -0.228];
B = [0.0223 0.547 0 -1.169]';
C = eye(4);
D = [0 0 0 0]';
G_lat = ss(A,B,C,D);

% Compute eigenstructure
[V,lambda] = eig(A);
V1 = V(:,1);    % Roll
V2 = V(:,2);    % Dutch Roll 1
V3 = V(:,3);    % Dutch Roll 2
V4 = V(:,4);    % Spiral    

figure(1); 
compass(V1);

damp(G_lat);    % or damp(A)
eig(A);
