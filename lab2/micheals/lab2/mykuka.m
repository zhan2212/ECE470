function myrobot = mykuka(DH_param)
    
    for i = 1:6
        links(i) = Revolute('d', DH_param(i,4), 'a', DH_param(i,2), 'alpha', DH_param(i,1));
    end
    
    myrobot = SerialLink(links, 'name', 'kuka');
end