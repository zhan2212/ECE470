function mypattern2(myrobot)
    X_workspace = zeros(3,100);
    x = linspace(0,2*pi,100);
    X_workspace(1,:) = 500+15*x;
    X_workspace(2,:) = 25*(cos(x)+sin(9*x));
    X_workspace(3,:) = -0.8;

    X_baseframe = zeros(3,100);

    for i=1:100
        X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
    end

    R = [0 0 1;0 -1 0; 1 0 0];
    for i=1:100
        H = [R X_baseframe(:,i); zeros(1,3) 1];
        q = inverse_kuka(H,myrobot);
        setAngles(q,0.02);
    end

end