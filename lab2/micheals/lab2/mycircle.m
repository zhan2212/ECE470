function X_baseframe = mycircle()
    % generate a circle
    angles = linspace(0,2*pi,100);
    radius = 50;
    X_workspace = ones(3,100);
    X_workspace(1,:) = 620 + radius * sin(angles);
    X_workspace(2,:) = 0 + radius * cos(angles);
    X_workspace(3,:) = X_workspace(3,:)*1.5;

    % transform every col of X_workspace into baseframe
    X_baseframe = zeros(3,100);
    for i = 1:size(X_workspace,2)
        X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
    end
end