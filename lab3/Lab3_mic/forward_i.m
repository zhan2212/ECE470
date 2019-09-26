function H = forward_i(joint_num, joint, myrobot)
    H = myrobot.A(1:joint_num,joint(1:joint_num)).T();
end