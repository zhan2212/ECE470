function H = forward(joint, myrobot)
% initialize Anonymous Function H
H_templete = @(theta,alpha,a,d)[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
	
% compute intermeida Hs
H01 = H_templete(joint(1), myrobot.alpha(1),myrobot.a(1),myrobot.d(1));
H12 = H_templete(joint(2), myrobot.alpha(2),myrobot.a(2),myrobot.d(2));
H23 = H_templete(joint(3), myrobot.alpha(3),myrobot.a(3),myrobot.d(3));
H34 = H_templete(joint(4), myrobot.alpha(4),myrobot.a(4),myrobot.d(4));
H45 = H_templete(joint(5), myrobot.alpha(5),myrobot.a(5),myrobot.d(5));
H56 = H_templete(joint(6), myrobot.alpha(6),myrobot.a(6),myrobot.d(6));

H = H01*H12*H23*H34*H45*H56; 
end