function X_baseframe = mypattern()
    % generate a circle
    angles = linspace(0,4*pi,500);
    a = 25;
    b = 10;
    c = 25;
    X_workspace = ones(3,size(angles,2));
    X_workspace(1,:) = 620 + (a+b)*cos(angles) - c * cos((a/b+1).*angles);
    X_workspace(2,:) = (a+b)*sin(angles) - c * sin((a/b+1).*angles);
    X_workspace(3,:) = 2*X_workspace(3,:);

    % transform every col of X_workspace into baseframe
    X_baseframe = zeros(size(X_workspace));
    for i = 1:size(X_workspace,2)
        X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
    end
    %plot(X_workspace(1,:), X_workspace(2,:));
end