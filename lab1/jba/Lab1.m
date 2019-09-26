theta1 = linspace(0, pi, 200);
theta2 = linspace(0, pi/2, 200);
theta3 = linspace(0, pi, 200);
theta4 = linspace(pi/4, 3*pi/4, 200);
theta5 = linspace(-pi/3, pi/3, 200);
theta6 = linspace(0, 2*pi, 200);
q = [theta1; theta2; theta3; theta4; theta5; theta6].'; 

DH = [pi/2 0 0 76; 0 43.23 0 -23.65; pi/2 0 0 0; -pi/2 0 0 43.18; pi/2 0 0 0; 0 0 0 20;];
myrobot = mypuma560(DH);
%4.2
% plot(myrobot, q)

%4.3
% o = zeros(200,3);
% for i = 1: 200
%     H = forward(q(i, :), myrobot);
%     o(i, :) = H(1:3,4).';
% end
% plot3(o(:,1), o(:,2), o(:,3), 'r')
% hold on
% plot(myrobot, q)

%4.4
% H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
% q = inverse(H, myrobot);



x = linspace(10,30,100);
y = linspace(23,30,100);
z = linspace(15,100,100);
R = [cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1;];
d = [x; y; z;].';
q = zeros(100,6);
for i = 1: 100
    H = [R d(i,:).'];
    H = [H; 0 0 0 1];
    q(i, :) = inverse(H, myrobot);
end
plot3(d(:,1), d(:,2), d(:,3), 'r')
hold on
plot(myrobot, q)

