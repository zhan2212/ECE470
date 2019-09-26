function myrobot= mykuka(DH)
alpha = DH(:,1); % Retrieve alpha from DH table
a = DH(:,2); % Retrieve a from DH table
theta = DH(:,3); % Retrieve theta from DH table
d = DH(:,4); % Retrieve d from DH table
myrobot = SerialLink([theta d a alpha], 'name', 'kukakr5'); % call SerialLink
end