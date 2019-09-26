function [obs] = setupobstacle424()
% Units are mm

% Obstacle 1: ground plane
obs{1}.rho0 = 150;
obs{1}.type = 'plane';

% Obstacle 2: Cylinder
obs{2}.R = 100;
obs{2}.c = [615;5];
obs{2}.rho0 = 150;
obs{2}.h = 572;
obs{2}.type = 'cyl';

% Obstacle 3: Cylinder
obs{3}.R = 100;
obs{3}.c = [625;-445];
obs{3}.rho0 = 150;
obs{3}.h = 572;
obs{3}.type = 'cyl';

end