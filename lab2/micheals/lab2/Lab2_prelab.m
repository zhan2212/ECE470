%alpha a theta d
DH = [
    pi/2 25 0 400;
    0 315 0 0;
    pi/2 35 0 0;
    -pi/2 0 0 365;
    pi/2 0 0 0;
    0 -296.23 0 161.44
];
kuka = mykuka(DH);
forw = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]', kuka)
inverse_kuka(forw, kuka)