function [sigma_points] = compute_sigma_points(mu, sigma)
% Computes the 2n+1 sigma points according to the unscented transform,
% where n is the dimensionality of the mean vector mu.
% The sigma points should form the columns of sigma_points,
% i.e. sigma_points is an nx2n+1 matrix.

global scale;

% Compute lambda
n = length(mu);

num_sig = 2*n+1;

lambda = scale - n;

% Condition sigma for point selction
transform_sig = (n + lambda) * sigma;

% Cholesky decomposition may be faster
sqrt_sig = chol(transform_sig, "lower");

% TODO: Compute sigma points
% Using ovtave functions
mu_points = repmat(mu, 1, n);
sigma_points = [mu, mu_points + sqrt_sig, mu_points - sqrt_sig];

% Normal code: result is the same but in diffrent column order
%sigma_points = zeros(n,num_sig);
%for i = 1:n
%sigma_points(:,i+1) = mu + sqrt_sig(:, i);
%sigma_points(:,2*(n+1)-i) = mu - sqrt_sig(:, i);
%endfor

end
