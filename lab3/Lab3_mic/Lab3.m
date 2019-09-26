%% initialize the dh parameters with 0 theta
dh = define_dh_for_puma([0 0 0 0 0 0]);

%% initialize robot with the dh parameters
myrobot = mypuma560(dh);

%% Section 3.1 The Attractive Field
H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
H1(1:3,4)=100*[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
q1 = inverse(H1,myrobot)
% This is the starting joint variable vector.
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
% This is the final joint variable vector
tau = att(q1,q2,myrobot)

%% Section 3.2 Motion Planning
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q);

%% Section 3.3 Motion Planning with Obstacle
setupobstacle
q3 = 0.9 * q1 + 0.1 * q2;
tau = rep(q3, myrobot, obs{1})
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q, myrobot, obs{6})

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
hold off;