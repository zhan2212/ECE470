function tau = rep(q, myrobot, obs)
weight = [1 1 1 1 1 1];
%preallocate for tau
tau = zeros(1,size(q,2));

% velocity jacobian mtx
prev_joint_position = zeros(3,size(q,2)+1);
prev_z = zeros(3,size(q,2)+1);
prev_z(3,1) = 1;

for i=1:size(tau,2)
    distance_to_obstacle = 0;
    vec_to_obstacle = zeros(3,1);
    global_pose_of_joint = forward_i(i,q,myrobot);
    if obs.type == 'sph'
        distance_to_obstacle = max(norm(obs.c-global_pose_of_joint(1:3,4)) - obs.R,0)
        vec_to_obstacle = (global_pose_of_joint(1:3,4) - obs.c)*...
            distance_to_obstacle/norm(global_pose_of_joint(1:3,4) - obs.c)
    else
        distance_to_obstacle = max(norm(obs.c-global_pose_of_joint(1:2,4)) - obs.R,0);
        vec_to_obstacle(1:2,1) = (global_pose_of_joint(1:2,4) - obs.c)*...
            distance_to_obstacle/norm(global_pose_of_joint(1:2,4) - obs.c);
    end
    
    force_i = zeros(3,1);
    if distance_to_obstacle <= obs.rho0
        force_i = weight(1,i) * (1/distance_to_obstacle-1/obs.rho0) / distance_to_obstacle^2 /distance_to_obstacle ...
        * vec_to_obstacle;
    end
    
    jacob_mtx = zeros(3,size(q,2));
    for j = 1 : i
        delta_position = global_pose_of_joint(1:3,4)-prev_joint_position(:,j);
        jacob_mtx(:,j) = cross(prev_z(:,j), delta_position);
    end
    tau = tau + transpose(jacob_mtx' * force_i);
    prev_joint_position(:,i+1) = global_pose_of_joint(1:3,4);
    prev_z(:,i+1) = global_pose_of_joint(1:3,3);
end
if(norm(tau)>0)
    tau = tau ./ norm(tau);
end
end