function [sigma_points] = motion_model(pose, u)

% get the size of the sigma points matrix
[n_sigma_points, m_sigma_points] = size(sigma_points);

% Apply the odometry model
pose(1) = pose(1) + u.t*cos(pose(3) + u.r1);
pose(2) = sigma_points(2,:) + u.t*sin(sigma_points(3,:) + u.r1);
sigma_points(3,:) = sigma_points(3,:) + u.r1 + u.r2;
sigma_points(3,:) = normalize_angle(sigma_points(3,:));

end