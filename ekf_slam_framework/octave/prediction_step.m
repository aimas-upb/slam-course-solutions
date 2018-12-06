function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

% Create command vector
[m,n] = size(sigma);

v = zeros(m,1);

v(1,1) = u.t*cos(mu(3) + u.r1);
v(2,1) = u.t*sin(mu(3) + u.r1);
v(3,1) = u.r1+u.r2;

% Compute noise free mean
mu = mu + v;

% Normalize angel, can also use the trick: atan2(sin(x),cos(x))
mu(3) = normalize_angle(mu(3));

% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gx = eye(3,3);
Gx(1,3) = -v(1);
Gx(2,3) = v(2);

% TODO: Construct the full Jacobian G
G = [Gx, zeros(3,n-3); zeros(n-3,3), eye(n-3, n-3)];

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion

% Compute Global Covariance Matrix
%sigma = G*sigma*G' + R;

% Update by Parts Only

% odometry
sigma_xx = sigma(1:3,1:3);

% landmarks
sigma_xm = sigma(1:3,3:n);

% compute covariances
sigma_xx = Gx*sigma_xx*Gx';
sigma_xm = Gx*sigma_xm;

% update global covariance
sigma(1:3, 1:3) = sigma_xx;
sigma(1:3,3:n) = sigma_xm;
sigma(3:n,1:3) = sigma_xm';

% add motion noise
sigma = sigma + R;


end
