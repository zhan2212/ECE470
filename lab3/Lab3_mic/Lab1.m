%% initialize the dh parameters with 0 theta
dh = define_dh_for_puma([0 0 0 0 0 0]);

%% initialize robot with the dh parameters
myrobot = mypuma560(dh);

%% Exercise 4.2

% initialize the joint space trajectory
q = transpose([ linspace(0,pi,200);
    linspace(0,pi/2,200);
    linspace(0,pi,200);
    linspace(pi/4,3*pi/4,200);
    linspace(-pi/3,pi/3,200);
    linspace(0,2*pi,200);
    ]);

% visualize the robot trajectories using simulation
plot(myrobot,q);

%% Exercise 4.3

o = zeros(size(q,1),3);

% for each row of q
for i = 1:size(q,1)
    % compute the end effector poses for each row of q
    row = q(i,:);
    H_matrix = forward(row, myrobot);
    
    % extract the position information from H_matrix
    o(i,:) = transpose(H_matrix.t);
end

% plot the visualizations
plot3(o(:,1), o(:,2), o(:,3), 'r');
hold on;
plot(myrobot,q);

%% Exercise 4.4 
% inverse function validation
H = [cos(pi/4) -sin(pi/4) 0 20;
    sin(pi/4) cos(pi/4) 0 23;
    0 0 1 15;
    0 0 0 1];
q = inverse(H, myrobot);

% actual activity
% initialize d matrix
d = zeros(100,3);
d(:,1) = transpose(linspace(10,30,100));
d(:,2) = transpose(linspace(23,30,100));
d(:,3) = transpose(linspace(15,100,100));

% initialize rotational R matrix
R = eye(3);
angle = pi/4;
R(1,1) = cos(angle);
R(1,2) = -sin(angle);
R(2,1) = sin(angle);
R(2,2) = cos(angle);

% determine q
q = zeros(100,6);

% for each row of d
for i = 1:size(d,1)
    row = d(i,:);
    %construct H matrix
    H_mat_intermediate = eye(4);
    H_mat_intermediate(1:3,1:3) = R;
    H_mat_intermediate(1:3,4) = transpose(row);
    q(i,:) = inverse(H_mat_intermediate, myrobot);
end
% plot the visualizations
plot3(d(:,1), d(:,2), d(:,3), 'r');
hold on;
plot(myrobot,q);
%% function declarations
function dh_params = define_dh_for_puma(theta)
    dh_params = [
        pi/2, 0, theta(1), 76;
        0, 43.23, theta(2), -23.65;
        pi/2, 0, theta(3), 0;
        -pi/2, 0, theta(4), 43.18;
        pi/2, 0, theta(5), 0;
        0, 0, theta(6), 20
        ];
end


