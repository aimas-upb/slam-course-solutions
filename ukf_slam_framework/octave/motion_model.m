function [sigma_points] = motion_model(sigma_points, u)

% get the size of the sigma points matrix
[n_sigma_points, m_sigma_points] = size(sigma_points);

% Apply the odometry model
sigma_points(1,:) = sigma_points(1,:) + u.t*cos(sigma_points(3,:) + u.r1);
sigma_points(2,:) = sigma_points(2,:) + u.t*sin(sigma_points(3,:) + u.r1);
sigma_points(3,:) = sigma_points(3,:) + u.r1 + u.r2;
sigma_points(3,:) = normalize_angle(sigma_points(3,:));

end