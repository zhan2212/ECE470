function q = inverse(H,myrobot)
% initalize aliases
d2 = myrobot.d(2);
a2 = myrobot.a(2);
d4 = myrobot.d(4);
d1 = myrobot.d(1);
Od = H(1:3,4);
Rd = H(1:3,1:3);

Oc = Od- Rd*[0;0;20];
Xc = Oc(1);
Yc = Oc(2);
Zc = Oc(3);

% solve for theta1, theta2, theta3
theta1 = atan2(Yc, Xc) - atan2(-d2, sqrt(Xc^2 + Yc^2 - d2^2));
D = (Xc^2 + Yc^2 - d2^2 + (Zc - d1)^2 - a2^2 - d4^2)/(2*a2*d4);
theta3 = atan2(D, sqrt(1 - D^2));
theta2 = atan2(Zc - d1, sqrt(Xc^2 + Yc^2 - d2^2)) - atan2(-d4*cos(theta3), a2 + d4*sin(theta3));

% compute H03
H_templete = @(theta,alpha,a,d)[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
H01 = H_templete(theta1, myrobot.alpha(1),myrobot.a(1),myrobot.d(1));
H12 = H_templete(theta2, myrobot.alpha(2),myrobot.a(2),myrobot.d(2));
H23 = H_templete(theta3, myrobot.alpha(3),myrobot.a(3),myrobot.d(3));
H3 = H01*H12*H23;

R6 = H3(1:3,1:3).'*Rd;

% solve for theta4, theta5, theta6
theta4 = atan2(R6(2,3), R6(1,3));
theta5 = atan2(sqrt(1-R6(3,3)^2), R6(3,3));
theta6 = atan2(R6(3,2), -R6(3,1));

% format result
q = [theta1 theta2 theta3 theta4 theta5 theta6];
end

