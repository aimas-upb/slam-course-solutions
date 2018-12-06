function [sigma_points, w_m, w_c] = compute_sigma_points(mu, sigma, lambda, alpha, beta)
% This function samples 2n+1 sigma points from the distribution given by mu and sigma
% according to the unscented transform, where n is the dimensionality of mu.
% Each column of sigma_points should represent one sigma point
% i.e. sigma_points has a dimensionality of nx2n+1.
% The corresponding weights w_m and w_c of the points are computed using lambda, alpha, and beta:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n] (i.e. each of size 1x2n+1)
% They are later used to recover the mean and covariance respectively.

n = length(mu);
sigma_points = zeros(n,2*n+1);

% TODO: compute all sigma points
% select first sigma point = the mean
sigma_points(:,1) = mu;

% compute matrix square root using cholesky factorization
sqrt_fact = chol((n+lambda)*sigma, "lower");

% compute the rest of the sigma points
for i = 1:n
  sigma_points(:, i+1) = mu + sqrt_fact(:, i);
  sigma_points(:, n+i+2) = mu - sqrt_fact(:, i);
endfor 

% TODO compute weight vectors w_m and w_c
w_m = zeros(1,2*n+1);
w_c = zeros(1,2*n+1);

% mean sigma point weights
w_m(1,1) = lambda/(n + lambda);
w_c(1,1) = w_m(1, 1) + (1 - alpha^2 + beta);

% generic sigma point weights
w_m(1, 2:end) = 1/(2*(n+lambda));
w_c(1, 2:end) = 1/(2*(n+lambda));

end
