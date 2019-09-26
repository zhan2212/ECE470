function myrobot= mykuka_search(delta)
DH_temp = @(theta1,theta2,theta3,theta4,theta5,theta6)[pi/2 25 theta1 400;
    0 315 theta2 0;
    pi/2 35 theta3 0;
    -pi/2 0 theta4 365;
    pi/2 0 theta5 0;
    0 -296.23 theta6 161.44];
DH = DH_temp(0,0,0,0,0,0);
alpha = DH(:,1); % Retrieve alpha from DH table
a = DH(:,2); % Retrieve a from DH table
theta = DH(:,3); % Retrieve theta from DH table
d = DH(:,4); % Retrieve d from DH table
a(6) = a(6) + delta(1);
d(6) = d(6) + delta(2);
myrobot = SerialLink([theta d a alpha], 'name', 'kukakr5'); % call SerialLink
end