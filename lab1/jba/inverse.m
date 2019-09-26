function q = inverse(H, myrobot)
%q = myrobot.ikine6s(H, 'lu');
d2 = -23.65;
a2 = 43.23;
d4 = 43.18;
d1 = 76;
Od = H(1:3,4);
Rd = H(1:3,1:3);
Oc = Od- Rd*[0;0;20];
Xc = Oc(1);
Yc = Oc(2);
Zc = Oc(3);
theta1 = atan2(Yc, Xc) - atan2(-d2, sqrt(Xc^2 + Yc^2 - d2^2));
D = (Xc^2 + Yc^2 - d2^2 + (Zc - d1)^2 - a2^2 - d4^2)/(2*a2*d4);
theta3 = atan2(D, sqrt(1 - D^2));
theta2 = atan2(Zc - d1, sqrt(Xc^2 + Yc^2 - d2^2)) - atan2(-d4*cos(theta3), a2 + d4*sin(theta3));
H3 = myrobot.A(1:3, [theta1, theta2, theta3]).T;
R6 = H3(1:3,1:3).'*Rd;
theta4 = atan2(R6(2,3), R6(1,3));
theta5 = atan2(sqrt(1-R6(3,3)^2), R6(3,3));
theta6 = atan2(R6(3,2), -R6(3,1));
q = [theta1 theta2 theta3 theta4 theta5 theta6];
end