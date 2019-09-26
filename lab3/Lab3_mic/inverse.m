function q = inverse(H, myrobot)
    d = myrobot.d;
    a = myrobot.a;

    % wrist position
    end_effector_to_wrist = eye(4);
    end_effector_to_wrist(3,4) = -d(6);
    H_wrist = H*end_effector_to_wrist;
    x = H_wrist(1,4);
    y = H_wrist(2,4);
    z = H_wrist(3,4);
    
    
    % determine theta1
    theta1 = atan2(y,x) - atan2(-d(2), real(sqrt(x^2 + y^2 - d(2)^2)));
    
    % determine theta3
    D_val = -(a(2)^2 + d(4)^2 - x^2 - y^2 -(z-d(1))^2 + d(2)^2)/2/a(2)/d(4);
    theta3 = atan2(D_val,real(sqrt(1-D_val^2)));
    
    % determine theta2
    phi_1 = atan2(d(4) * sin(theta3 - pi/2), a(2) + d(4) * cos(theta3 - pi/2));
    phi_2 = atan2(z-d(1), real(sqrt(x^2 + y^2 - d(2)^2)));
    theta2 = phi_2 - phi_1;
    
    % determine theta4,5,6
    H_3_in_frame_0 = myrobot.A(1:3,[theta1 theta2 theta3]);
    H_6_in_frame_3 = H_3_in_frame_0.T() \ H;
    theta4 = atan2(H_6_in_frame_3(2,3), H_6_in_frame_3(1,3));
    theta5 = atan2(real(sqrt(1-H_6_in_frame_3(3,3)^2)), H_6_in_frame_3(3,3));
    theta6 = atan2(H_6_in_frame_3(3,2), -H_6_in_frame_3(3,1));
    
    q = [theta1, theta2, theta3, theta4, theta5, theta6];

end