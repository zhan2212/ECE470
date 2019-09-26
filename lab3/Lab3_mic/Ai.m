% This function is specific to the Puma 560 robot.

function A = Ai(theta,link)

    RotTheta = [cos(theta) -sin(theta) 0 0;...
             sin(theta) cos(theta) 0 0;...
             0 0 1 0; 0 0 0 1;];

    d = link.d;
    TransD = eye(4,4);
    TransD(3,4) = d;

    a = link.a;
    TransA = eye(4,4);
    TransA(1,4) = a;

    alpha = link.alpha;
    RotAlpha = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0;...
             0 sin(alpha) cos(alpha) 0; 0 0 0 1;];

    A = RotTheta*TransD*TransA*RotAlpha;
        
end











