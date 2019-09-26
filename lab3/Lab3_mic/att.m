function tau = att(q,q2,myrobot)
% q acutal joint angles
% q2 final joint angles
% myrobot robot

% weight vector
weight = [1 1 1 1 1 1];

%preallocate for tau
tau = zeros(1,size(q,2));

% velocity jacobian mtx
prev_joint_position = zeros(3,size(q,2)+1);
prev_z = zeros(3,size(q,2)+1);
prev_z(3,1) = 1;

for i=1:size(tau,2)
    global_pose_of_joint = forward_i(i,q,myrobot);
    global_pose_of_desired = forward_i(i,q2,myrobot);
    force_i = - weight(1,i) * (global_pose_of_joint(1:3,4) - global_pose_of_desired(1:3,4));
      
    jacob_mtx = zeros(3,size(q,2));
    for j = 1 : i
        delta_position = global_pose_of_joint(1:3,4)-prev_joint_position(:,j);
        jacob_mtx(:,j) = cross(prev_z(:,j), delta_position);
    end
    tau = tau + transpose(jacob_mtx' * force_i);
    prev_joint_position(:,i+1) = global_pose_of_joint(1:3,4);
    prev_z(:,i+1) = global_pose_of_joint(1:3,3);
end
tau = tau ./ norm(tau);
end