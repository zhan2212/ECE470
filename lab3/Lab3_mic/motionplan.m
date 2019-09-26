function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    q = [q0];
    alpha = 0.03;
    while norm(q(end,1:5) - q2(1,1:5))>tol
        att_f = att(q(end,:),q2, myrobot);
        new_q = q(size(q,1),:) + alpha * att_f/norm(att_f);
        for num = 1:numel(obs)
            rep_f = rep(q(end,:),myrobot, obs{num});
            if norm(rep_f)>0
                new_q = new_q + alpha * rep_f/norm(rep_f);
            end
        end
        new_q
        q = [q; new_q];
    end
    q(:,6) = linspace(q0(6),q2(6),size(q,1));
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');
    
end