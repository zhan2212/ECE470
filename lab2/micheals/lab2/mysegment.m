function X_baseframe = mysegment()
    % generate a line parallel to y 
    X_workspace = ones(3,100);
    X_workspace(2,:) = linspace(-100,100,100);
    X_workspace(1,:) = X_workspace(1,:)*620;
    X_workspace(3,:) = X_workspace(3,:)*2;

    % transform every col of X_workspace into baseframe
    X_baseframe = zeros(3,100);
    for i = 1:size(X_workspace,2)
        X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
    end
end