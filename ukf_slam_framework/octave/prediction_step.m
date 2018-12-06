function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% For computing lambda.
global scale;

% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);

% Dimensionality
n = length(mu);

% lambda
lambda = scale - n;

% TODO: Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles
sigma_points = motion_model(sigma_points, u);

% TODO: Compute the weights of the sigma points and recover mu.
% Be careful when computing the robot's orientation.

% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;

% compute the mean
cosines = sum(cos(sigma_points(3,:)).*wm);
sines = sum(sin(sigma_points(3,:)).*wm);
mu = sum(sigma_points .* repmat(wm, rows(sigma_points), 1), 2);
mu(3)   = atan2(sines, cosines);

% TODO: Recover sigma
d = sigma_points .- mu;
d(3,:) = normalize_angle(d(3,:));
sigma = d*diag(wc)*d';

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Add motion noise to sigma
sigma = sigma + R;
end
