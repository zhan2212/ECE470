function H = forward_kuka( myrobot,q)
    H = myrobot.A(1:6,q);
    H = H.T();
end