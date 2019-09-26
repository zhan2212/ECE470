
%%
% 4.1 
% initialize Anonymous Function DH
DH = @(theta1,theta2,theta3,theta4,theta5,theta6)[pi/2 25 theta1 400;
    0 315 theta2 0;
    pi/2 35 theta3 0;
    -pi/2 0 theta4 365;
    pi/2 0 theta5 0;
    0 -296.23 theta6 161.44];

% initialize myrobot
kuka = mykuka(DH(0,0,0,0,0,0));

%%
% prelab
ans = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]',kuka);

%%
syms t1 t2 t3 a1 a2 a3 d4 d1 d2 xc yc zc;
D = a1+a2*cos(t2)+d4*sin(t2+t3)+a3*cos(t2+t3);
eq1=cos(t1)*D==xc;
eq2=sin(t1)*D==yc;
eq3=d1+a3*sin(t2+t3)+d2*sin(t2)-d4*cos(t2+t3)==yc;
[solt1,solt2,solt3]=solve(eq1,eq2,eq3,t1,t2,t3)

