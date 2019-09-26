% This function is specific to the Puma 560 robot.

function q = inverse_ta(H,myrobot)
        
    config.LR = 'L';    %L or R
    config.UD = 'U';    %U or D
    config.W = 'P';     %P or N
    
    q = zeros(1,6);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Preliminary Work
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    ox = H(1,4);
    oy = H(2,4);
    oz = H(3,4);
    R = H(1:3,1:3);
 
    for i=1:6
       linktemp = myrobot.links(i);
       a(i) = linktemp.a;
       alpha(i) = linktemp.alpha;
       d(i) = linktemp.d;
    end
    
    xc = ox - d(6)*R(1,3);    %(3.35)
    yc = oy - d(6)*R(2,3);    %(3.35)
    zc = oz - d(6)*R(3,3);    %(3.35)     

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inverse Position Kinematics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%% theta1 %%%%%%%%%%%
    switch(config.LR)
        case 'L'
            r1 = real(sqrt(xc^2+yc^2));
            t1 = real(sqrt(r1^2-d(2)^2));
            phi1 = atan2(yc,xc);
            alpha1 = atan2(-d(2),t1);
            q(1) = phi1-alpha1;
    end


    %%%%%%% theta2 and theta3 %%%%%%%%%%%
    switch(config.UD)
        case 'U'
            D3 = (xc^2+yc^2-d(2)^2+(zc-d(1))^2-a(2)^2-d(4)^2)/(2*a(2)*d(4));
            q(3) = atan2(D3,real(sqrt(1-D3^2)));
    end

    alpha2 = atan2(d(4)*cos(q(3)),a(2)+d(4)*sin(q(3)));
    phi2 = atan2(zc-d(1),real(sqrt(xc^2+yc^2-d(2)^2)));
    q(2) = phi2 + alpha2;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inverse Orientation Kinematics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%% R03 %%%%%%%%%%%%%
    H = eye(4,4);
    for j=1:3
        H = H*Ai(q(j),myrobot.links(j));
    end
    R03 = H(1:3,1:3);

    R36 = R03'*R;   %(3.53)

    %%%%%%% theta5 %%%%%%%%%%%  
    switch(config.W)
        case 'P'            
            q(5) = atan2(sqrt(1-R36(3,3)^2),R36(3,3));
         case 'N'
            q(5) = atan2(-sqrt(1-R36(3,3)^2),R36(3,3));
    end

    %%%%%%% theta4 and theta 6 %%%%%%%%%%%  
    if (round(q(5)*10^10)*10^(-10))==0
        %in a singular configuration
        switch(config.W)
            case 'P'
                    q(4) = 0; %arbitrarily pick theta4 = 0
                    q(6) = atan2(R36(2,1),R36(1,1));
                    
            case 'N'
                
        end
    else
        switch(config.W)
            case 'P'
                q(4) = atan2(R36(2,3),R36(1,3));
                q(6) = atan2(R36(3,2),-R36(3,1));
            case 'N'
                
        end
    end
end

