function H = forward(joint, myrobot)
H = myrobot.A(1:6, joint).T;
end
