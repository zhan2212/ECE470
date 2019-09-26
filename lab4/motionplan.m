function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    % define alpha and q 
    alpha_att = 0.013;
    alpha_rep = 0.01;
    q = [];
    q(1,:) = q0;
    
    % gradient descent
    while norm(q(end,1:5)-q2(1:5))>=tol
        % obtain next q based on attractive force
        att_force = att(q(end,:), q2, myrobot);
        next_q = q(size(q,1),:) + alpha_att * att_force/norm(att_force);

        % obtain next q based on repulsive force
        for i = 1:numel(obs)
            rep_force = rep(myrobot, q(end,:), obs{i});
            if norm(rep_force) > 0
                next_q = next_q + alpha_rep * rep_force/norm(rep_force);
            end
        end
        % append next_q
        q = [q; next_q];
    end
    % construct qref
    q(:,6) = linspace(q0(6),q2(6),size(q,1));
    t = linspace(t1, t2, size(q,1));
    qref = spline(t,q');
end