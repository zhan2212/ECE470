clear all
close all
clc

%%
DH = @(theta1,theta2,theta3,theta4,theta5,theta6)[pi/2 25 theta1 400;
    0 315 theta2 0;
    pi/2 35 theta3 0;
    -pi/2 0 theta4 365;
    pi/2 0 theta5 0;
    0 156 theta6 161.44];

DH_forces = @(theta1,theta2,theta3,theta4,theta5,theta6)[pi/2 25 theta1 400;
    0 315 theta2 0;
    pi/2 35 theta3 0;
    -pi/2 0 theta4 365;
    pi/2 0 theta5 0;
    0 0 theta6 161.44];

myrobot1 = mykuka(DH(0,0,0,0,0,0));
myrobot2 = mykuka(DH_forces(0,0,0,0,0,0));
%%
prepobs{1}.R = 100;
prepobs{1}.c = [250; 0];
prepobs{1}.rho0 = 500;
prepobs{1}.h = 300;
prepobs{1}.type = 'cyl';

%%
tau = rep(myrobot2,[pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6],prepobs{1});


%%
kuka = mykuka(DH(0,0,0,0,0,0));
kuka_forces = mykuka(DH_forces(0,0,0,0,0,0));
p1 = [620 375 50];
p2 = [620 -375 50];
R=[0 0 1;0 -1 0;1 0 0];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
q1 = inverse_kuka(kuka, H1);
q2 = inverse_kuka(kuka, H2);
qref = motionplan(q1, q2,0,10, kuka_forces, prepobs,0.05);

%%
plotobstacle(prepobs);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(kuka,q);
hold off

