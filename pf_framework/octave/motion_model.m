function [x] = motion_model(x, u, noise)
% Updates the robot pose according to the motion model
% x: 3x1 vector representing the particle pose [x; y; theta]
% u: struct containing odometry reading (r1, t, r2).
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

%x(1) = normrnd(x(1) + u.t*cos(x(3) + u.r1),noise(1));
%x(2) = normrnd(x(2) + u.t*sin(x(3) + u.r1), noise(2));
%x(3) = normrnd(normalize_angle(x(3) + u.r1 + u.r2), noise(2));


% or using additive noise within k sigma 
% (k = 1 covers ~70% of normal distrib)
% (k = 1 covers ~95% of normal distrib)
% (k = 3 covers ~99% of normal distrib)
k = 1;
x(1) = x(1) + u.t*cos(x(3) + u.r1) + unifrnd(-k*noise(1), k*noise(1));
x(2) = x(2) + u.t*sin(x(3) + u.r1) + unifrnd(-k*noise(2), k*noise(2));
x(3) = normalize_angle(x(3) + u.r1 + u.r2 + unifrnd(-k*noise(3), k*noise(3)));
end
