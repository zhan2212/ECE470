function mypattern(myrobot)
    data=xlsread('jug.xlsx');
    xdata=550 + 10*data(:,1);
    ydata=10*data(:,2);
    zdata=-ones(length(data),1);

    X_workspace = zeros(3,numel(xdata));
    X_workspace(1,:) = xdata';
    X_workspace(2,:) = ydata';
    X_workspace(3,:) = zdata';

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