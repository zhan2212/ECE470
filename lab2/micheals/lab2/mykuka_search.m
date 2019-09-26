function myrobot = mykuka_search(delta)
    DH = [
    pi/2 25 0 400;
    0 315 0 0;
    pi/2 35 0 0;
    -pi/2 0 0 365;
    pi/2 0 0 0;
    0 -296.23 0 161.44
    ];
    
    DH(6,2) = DH(6,2)+delta(1);
    DH(6,4) = DH(6,4)+delta(2);
    for i = 1:6
        links(i) = Revolute('d', DH(i,4), 'a', DH(i,2), 'alpha', DH(i,1));
    end
    
    myrobot = SerialLink(links, 'name', 'kuka');
end