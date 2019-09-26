%% Session 3.1
% initialize Anonymous Function DH
DH = @(theta1,theta2,theta3,theta4,theta5,theta6)[pi/2 0 theta1 76;
    0 43.23 theta2 -23.65;
    pi/2 0 theta3 0;
    -pi/2 0 theta4 43.18;
    pi/2 0 theta5 0;
    0 0 theta6 20];

% initialize myrobot
myrobot = mypuma560(DH(0,0,0,0,0,0));
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4)=100*[-1; 3; 3;]/4;
q1 = inverse(H1,myrobot);
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
tau = att(q1,q2,myrobot)

%% Session 3.2
qref = motionplan(q1,q2,0,10,myrobot,[],0.1);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)


%% Session 3.3

%% using q1 and q2
setupobstacle
q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1}) % This tests the torque for the cylinder obstacle

%% using the sphere obstacle
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6})

%% motion planning with obstacle
setupobstacle
hold on
axis([-1 1 -1 1 0 2]*100)
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.1);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off