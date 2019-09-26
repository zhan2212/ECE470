%% Exercise 4.2 Calibration
%% Initialize Robot
% data collected for calibration
Q1 = [-0.1747    1.0324   -0.7943   -0.1806    1.3366    0.0431];
X1 = [464.36 -56.05 27]'; 
Q2 = [0.1595    0.8460   -0.3604   -0.0379    1.0208         0];
X2 = [571.45 88.13 22.60]';
Q3 = [0.2749    0.9242   -0.4829   -0.0379    1.0208         0];
X3 = [512.64 140.71 23.49]';

% optimization to calibrate
delta = fminunc(@deltajoint, [0 0])

% make a robot using the calibrated value
myrobot = mykuka_search(delta);

% verification
H = [ 0 0 1 0;
    0 -1 0 0;
    1 0 0 0;
    0 0 0 1];
H(1:3,4) = X3
%% run
q = inverse_kuka(H,myrobot)
setAngles(q, 0.04);

%% Exercise 4.3

p_workspace = [600; 100; 10];                       % target point
p_baseframe = FrameTransformation(p_workspace);      % transformed
R = [0 0 1; 0 -1 0; 1 0 0];                         % desired orientation
H = [R p_baseframe; zeros(1,3) 1];                  % homogeneous transformation
q = inverse_kuka(H,myrobot)                         % inverse kinematics calculation
setAngles(q,0.04);

%% Exercise 4.4

%% draw line segement
X_baseframe = mysegment();

% set angle operation main loop
for i = 1:size(X_baseframe,2)
    % homogeneous transformation
    H = [R X_baseframe(:,i); zeros(1,3) 1];   
    % inverse kinematics calculation
    q = inverse_kuka(H,myrobot)                         
    setAngles(q,0.04);
end

%% draw circle
X_baseframe = mycircle();

% set angle operation main loop
for i = 1:size(X_baseframe,2)
    % homogeneous transformation
    H = [R X_baseframe(:,i); zeros(1,3) 1];   
    % inverse kinematics calculation
    q = inverse_kuka(H,myrobot)                         
    setAngles(q,0.04);
end

%% draw jug
X_baseframe = jugpattern();

% set angle operation main loop
for i = 1:size(X_baseframe,2)
    % homogeneous transformation
    H = [R X_baseframe(:,i); zeros(1,3) 1];   
    % inverse kinematics calculation
    q = inverse_kuka(H,myrobot)                         
    setAngles(q,0.04);
end

%% my pattern
X_baseframe = mypattern();

% set angle operation main loop
for i = 1:size(X_baseframe,2)
    % homogeneous transformation
    H = [R X_baseframe(:,i); zeros(1,3) 1];   
    % inverse kinematics calculation
    q = inverse_kuka(H,myrobot)                         
    setAngles(q,0.04);
end