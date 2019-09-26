function tau = att(q,q2,myrobot)
% initialize the attractive force part of gradient descent
tau = zeros(1,6);
% initialize the weights
w = ones(1,6);
% predefine the variable to save the previous values
z_prev = zeros(3,7);
z_prev(3,1) = 1;
H_prev = zeros(3,7);

% compute the tau for each joint
for i = 1:6
    H = myrobot.A(1:i,q(1:i)).T();
    H_desired = myrobot.A(1:i,q2(1:i)).T();
    F_attractive = (H_desired(1:3,4) - H(1:3,4))*w(i);
    
    % compute the Jacobian matrix
    J = zeros(3,6);
    for j = 1 : i
        J(:,j) = cross(z_prev(:,j), H(1:3,4)-H_prev(:,j));
    end
    tau = tau + F_attractive'*J;
    z_prev(:,i+1) = H(1:3,3);
    H_prev(:,i+1) = H(1:3,4);
end
% normalize the output
tau = tau/norm(tau);
end