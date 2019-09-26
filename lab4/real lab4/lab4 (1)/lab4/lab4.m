%% Initialize DH table template
DH = @(theta1,theta2,theta3,theta4,theta5,theta6)[pi/2 25 theta1 400;
    0 315 theta2 0;
    pi/2 35 theta3 0;
    -pi/2 0 theta4 365;
    pi/2 0 theta5 0;
    0 -156 theta6 161.44];

DH_forces = @(theta1,theta2,theta3,theta4,theta5,theta6)[pi/2 25 theta1 400;
    0 315 theta2 0;
    pi/2 35 theta3 0;
    -pi/2 0 theta4 365;
    pi/2 0 theta5 0;
    0 0 theta6 161.44];

%% 4.1.1
p0 = [370 -440 150];
p1 = [370 -440 45];
p2 = [750 -220 225];
p3 = [620 350 225];

%% 4.1.2
% construct obs
obs = setupobstacle412();

%% 4.1.3
%done by changing parameters at motionplan and rep

%% 4.1.4
kuka = mykuka(DH(0,0,0,0,0,0));
kuka_forces = mykuka(DH_forces(0,0,0,0,0,0));
R = [0 0 1; 0 -1 0; 1 0 0];

H0=[R p0';zeros(1,3) 1];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
H3=[R p3';zeros(1,3) 1];

qh = [0 1.5708 0 0 1.5708 0]; % q for home position, it is obtained by getAngles() when robot is at its home position.
q0 = inverse_kuka(kuka, H0);
q1 = inverse_kuka(kuka, H1);
q2 = inverse_kuka(kuka, H2);
q3 = inverse_kuka(kuka, H3);

% compute q from home position to p0, p0 to p1, p1 to p2 and p2 to p3.
qh_0 = motionplan(qh, q0,0,10, kuka_forces, obs,0.01,0.01,0.013);
t=linspace(0,10,300);
qh_0=ppval(qh_0,t)';

q0_1 = motionplan(q0, q1,0,10, kuka_forces, obs,0.01,0.01,0.013);
t=linspace(0,10,300);
q0_1=ppval(q0_1,t)';

q1_2 = motionplan(q1, q2,0,10, kuka_forces, obs,0.01,0.01,0.013);
t=linspace(0,10,300);
q1_2=ppval(q1_2,t)';

q2_3 = motionplan(q2, q3,0,10, kuka_forces, obs,0.01,0.01,0.013);
t=linspace(0,10,300);
q2_3=ppval(q2_3,t)';

% plot out the result
hold on;
axis([-1500 1500 -1500 1500 0 1000]);
view(-32,50);
plotobstacle(obs);
plot(kuka, [qh_0;q0_1;q1_2;q2_3]);
hold off;


%% 4.2.1
setHome(0.04);

%% 4.2.2
% physically putting object in place

%% 4.2.3
% send out command to robot
setGripper(0);
for i=1:size(qh_0,1)
    setAngles(qh_0(i,:),0.04);
end
setAngles(q1,0.04);
setGripper(1);
for i=1:size(q1_2,1)
    setAngles(q1_2(i,:),0.04);
end
for i=1:size(q2_3,1)
    setAngles(q2_3(i,:),0.04);
end
setGripper(0);

%% 4.2.4
obs424 = setupobstacle424();

% compute q from home position to p0, p0 to p1, p1 to p2 and p2 to p3.
qh_0 = motionplan(qh, q0,0,10, kuka_forces, obs424,0.01,0.01,0.013);
t=linspace(0,10,300);
qh_0=ppval(qh_0,t)';

q0_1 = motionplan(q0, q1,0,10, kuka_forces, obs424,0.01,0.01,0.013);
t=linspace(0,10,300);
q0_1=ppval(q0_1,t)';

q1_2 = motionplan(q1, q2,0,10, kuka_forces, obs424,0.01,0.01,0.013);
t=linspace(0,10,300);
q1_2=ppval(q1_2,t)';

q2_3 = motionplan(q2, q3,0,10, kuka_forces, obs424,0.01,0.01,0.013);
t=linspace(0,10,300);
q2_3=ppval(q2_3,t)';

% send out command to robot
setGripper(0);
for i=1:size(qh_0,1)
    setAngles(qh_0(i,:),0.04);
end
setAngles(q1,0.04);
setGripper(1);
for i=1:size(q1_2,1)
    setAngles(q1_2(i,:),0.04);
end
for i=1:size(q2_3,1)
    setAngles(q2_3(i,:),0.04);
end
setGripper(0);

%% 4.2.5
% There are 4 paramteres in the algorithm: repulsive learning rate, attractive learning rate, tolerence, and rho.
% Repulsive and attractive learning rates affect how the robot should behave when encountering obstacle and trying to reach desire position.
% Larger learning rate means more aggressive reaction. i.e. Larger repulsive learning rate leads to that the robot trys to avoid obstacle further;
% larger attractive learning means that the robot has higher tendency to approach the desire position.
% Tuning learning rate is crucial to make the algorithm converge: Large learning rate will cause the problem that q jumps back and forth around the optimal minima but fail to reach it
% ; small learning rate might cause the problem that it stucks in some non-optimal local minima.
% Tolerence is the condition for the algorithm to stop.
% rho is the region of effect of each obstacle. Large rho might cause the robot to suffer from too much disturbance even if the robot is far away
% from the obstacle. Small rho will cause the problem that the robot can only react when it is close to the obstacle, which is risky.

%% 4.3
% summary of our plan: grip one more object located at p4 and place it to p3 after performing task stated in 4.2.3.
% note that we move robot from p3 to a point right above p4, namely, p4top; then reach p4 from p4top in order to grip the object successfully.

% compute q4top and q4
p4top = [411.6007 485.5296 400];
H4top=[R p4top';zeros(1,3) 1];
q4top = inverse_kuka(kuka, H4top);

p4 = [411.6007 485.5296 45];
H4=[R p4';zeros(1,3) 1];
q4 = inverse_kuka(kuka, H4);

% compute q from p3 to p4top, p4top to p4, p4 to p3.
q3_4top = motionplan(q3, q4top,0,5, kuka_forces, obs,0.01,0.01,0.013);
t=linspace(0,10,150);
q3_4top=ppval(q3_4top,t)';

q4top_4 = motionplan(q4top, q4,0,5, kuka_forces, obs,0.01,0.01,0.013);
t=linspace(0,10,150);
q4top_4=ppval(q4top_4,t)';

q4_3 = motionplan(q4, q3,0,5, kuka_forces, obs,0.01,0.01,0.013);
t=linspace(0,10,300);
q4_3=ppval(q4_3,t)';

% send out command to the robot
setGripper(0);
for i=1:size(qh_0,1)
    setAngles(qh_0(i,:),0.04);
end
setAngles(q1,0.04);
setGripper(1);
for i=1:size(q1_2,1)
    setAngles(q1_2(i,:),0.04);
end
for i=1:size(q2_3,1)
    setAngles(q2_3(i,:),0.04);
end
setGripper(0);
for i=1:size(q3_4top,1)
    setAngles(q3_4top(i,:),0.004);
end
for i=1:size(q4top_4,1)
    setAngles(q4top_4(i,:),0.004);
end
setGripper(1);
for i=1:size(q4_3,1)
    setAngles(q4_3(i,:),0.04);
end
setGripper(0);


