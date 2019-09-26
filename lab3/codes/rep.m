function tau = rep(q, myrobot, obs)
% initialize the weights
w = ones(1,6);
% initialize the repulsive force part of gradient descent
tau = zeros(1,6);
% predefine the variable to save the previous values
H_prev = zeros(3,7);
z_prev = zeros(3,7);
z_prev(3,1) = 1;

for i=1:6
    vect = zeros(3,1);
    H = myrobot.A(1:i,q(1:i)).T();
    
    % identify the type of obstacles
    if strcmp(obs.type,'cyl')
        dist = max(norm(obs.c-H(1:2,4))-obs.R, 0);
        vect(1:2,1) = (H(1:2,4) - obs.c) * dist / norm(H(1:2,4)-obs.c);
    else
        dist = max(norm(obs.c-H(1:3,4))-obs.R, 0);
        vect = (H(1:3,4)-obs.c) * dist / norm(H(1:3,4)-obs.c);
    end
    
    % compute the repulsive force
    F_repulsive = zeros(3,1);
    if dist <= obs.rho0
        F_repulsive = (1/dist-1/obs.rho0) / dist^2 / dist * vect * w(i) ;
    end
    
    % compute the Jacobian matrix
    J = zeros(3,6);
    for j = 1:i
        J(:,j) = cross(z_prev(:,j), H(1:3,4) - H_prev(:,j));
    end
    tau = tau + F_repulsive' * J;
    H_prev(:,i+1) = H(1:3,4);
    z_prev(:,i+1) = H(1:3,3);
end

% normalize the output
if norm(tau) ~= 0
    tau = tau / norm(tau);
end

end