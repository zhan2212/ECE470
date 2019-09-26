function mysegment(myrobot)
    X_workspace = zeros(3,100);
    X_workspace(1,:) = 620;
    X_workspace(2,:) = linspace(-100,100,100);
    X_workspace(3,:) = -0.5;

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