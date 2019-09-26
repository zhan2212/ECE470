function dh_params = define_dh_for_puma(theta)
    dh_params = [
        pi/2, 0, theta(1), 76;
        0, 43.23, theta(2), -23.65;
        pi/2, 0, theta(3), 0;
        -pi/2, 0, theta(4), 43.18;
        pi/2, 0, theta(5), 0;
        0, 0, theta(6), 20
        ];
end