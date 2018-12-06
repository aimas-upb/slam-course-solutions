function [z_points] = observation_model(sigma_points, map, id) 
  % Compute landmark index
  lidx = find(map == id);
  
  % Select landmark sigma points
  landmark_points = [sigma_points(2*lidx + 2, :); sigma_points(2*lidx + 3,:)];
  
  % Select odometry sigma points
  odom_points = [sigma_points(1,:); sigma_points(2,:)];
  
  % Propagate sigma points through measurement function h, i.e. compute z_points
  delta = landmark_points - odom_points; %[dx dy]^T where dx,dy(i) = 1:2n+1
  z_points = [sqrt(delta(1,:).^2 + delta(2,:).^2); atan2(delta(2,:), delta(1,:)) - sigma_points(3,:)];
end
