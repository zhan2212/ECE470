function X_baseframe = jugpattern()
    % load jug pattern
    data = xlsread('jug.xlsx');
    xdata = 550 + 10*data(:,1);
    ydata = 10 * data(:,2);
    zdata = zeros(length(data),1);
    
    % generate a line parallel to y 
    X_workspace = [xdata';ydata';zdata'];
    
    % transform every col of X_workspace into baseframe
    X_baseframe = zeros(size(X_workspace));
    for i = 1:size(X_workspace,2)
        X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
    end
end