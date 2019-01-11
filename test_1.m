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
i = 1
while i>0
    
    n1 = randi([-1 1],1,1)
    p1 = randi([-10 0],1,1) + (n1)*i;
    p2 = randi([-10 0],1,1) - (n1)*i;
    p3 = -0.01 ;
    p4 = -0.04 ;
    p5 = -0.03;
    p6 = -0.02;
    p7 = [p1 p2 p3 p4 p5 p6];
    Ko = place(A,B,p7);
    s2 = ss(A-B*Ko,B,C,D);
    t = 0:0.01:700;
    t1 = t';
    q = [0 0 0 0];
    r = ones(70000,4);
    u = [q;r];
    x0 = [0 0 0 0 0 0]'
    figure(2);
    ls1 = lsim(s2,u,t,x0);
    grid on;
    for j = 1:70001
        if ls1(j,3)<=1.5 && ls1(j,3)>=0.5 
            p1;
            p2;
            break;
        end
    end
    i=i+1;
end