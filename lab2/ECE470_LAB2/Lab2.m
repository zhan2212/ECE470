
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
% 4.2
% parameter obtained
X1=[644.06 -127.06 28.29]';
X2=[601.07 -56.69 27.84]';
X3=[634.06 -18.51 28.20]';  
Q1=[-0.1873    0.6842   -0.3672    0.0232    1.5743   -0.0051];
Q2=[-0.0194    0.7421   -0.5367    0.1836    1.6893   -0.0367];
Q3=[0.0777    0.7044   -0.4229    0.2747    1.6207   -0.0768];

% find delta by using fminuc
delta = fminunc(@deltajoint,[0 0]); % result: -2.4047   -7.2472

% construct robot model by using obtained delta
myrobot = mykuka_search(delta);

% verfy
H = [ 0 0 1 0;
    0 -1 0 0;
    1 0 0 0;
    0 0 0 1];
H(1:3,4) = X2; % construct H
q = inverse_kuka(H,myrobot); % compute inverse kinematics
setAngles(q, 0.04); %send command to robot


%%
% 4.3
p_workspace = [600; 100; 10]; % set up target point
p_baseframe = FrameTransformation(p_workspace); % frame transformation
R = [0 0 1; 0 -1 0; 1 0 0]; % set up desired orientation
H = [R p_baseframe; zeros(1,3) 1]; % construct homogeneous transformation
q = inverse_kuka(H,myrobot) % compute inverse kinematics
setAngles(q,0.04); %send command to robot

%%
% 4.4
% draw line segment
mysegment(myrobot);

% draw circle
mycircle(myrobot);

% draw jug
mypattern(myrobot);

% draw my own pattern
mypattern2(myrobot);

