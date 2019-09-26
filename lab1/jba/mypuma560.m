function myrobot = mypuma560(DH)
alpha = DH(:,1);
a = DH(:,2);
theta = DH(:,3);
d = DH(:,4);
myrobot = SerialLink([theta d a alpha], 'name', 'puma560');
end


