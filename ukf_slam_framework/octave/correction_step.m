function [mu, sigma, map] = correction_step(mu, sigma, z, map);
% Updates the belief, i.e., mu and sigma after observing landmarks,
% and augments the map with newly observed landmarks.
% The employed sensor model measures the range and bearing of a landmark
% mu: state vector containing robot pose and poses of landmarks obeserved so far.
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector 'map' contains the ids of all landmarks observed so far by the robot in the order
% by which they were observed, NOT in ascending id order.

% For computing sigma
global scale;

% Number of measurements in this time step
m = size(z, 2);

% Measurement noise
Q = 0.01*eye(2);

for i = 1:m
	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;

	% Get the actual measurement made
	z_actual = [z(i).range; z(i).bearing];

	% If the landmark is observed for the first time:
	if(isempty(find(map==landmarkId)))
  
	    % Add it to the map
	    %map = [map; landmarkId];
	    
      % TODO: Initialize its pose according to the measurement and add it to mu
      % Init landmark position to the robot position
      %mu = [mu; mu(1); mu(2)];
      
      % Compute size of mu to get the index
      %[n_mu m_mu] = size(mu);
      
      % Update landmak position
      %mu(n_mu - 1) = mu(n_mu - 1) + z(i).range*cos(z(i).bearing + mu(3)); 
      %mu(n_mu) = mu(n_mu) + z(i).range*sin(z(i).bearing+mu(3));

      % TODO: Initialize its uncertainty and add it to sigma
      % Make room for this landmark uncertainty
      %sigma = resize(sigma, n_mu, n_mu);
      
      % Add observation uncertainty
      %sigma(n_mu -1: n_mu, n_mu -1:n_mu) = sigma(n_mu -1: n_mu, n_mu -1:n_mu) + Q;
      
      [mu, sigma, map] = add_landmark_to_map(mu, sigma, z(i), map, Q);
      
      
	    continue;
	endif

	% Compute sigma points from the predicted mean and covariance
	sigma_points = compute_sigma_points(mu, sigma);
  % Normalize!
	sigma_points(3,:) = normalize_angle(sigma_points(3,:));
  
	% Compute lambda
	n = length(mu);
	num_sig = size(sigma_points,2);
	lambda = scale - n;

	% TODO: Compute z_points (2x2n+1), which consists of predicted measurements from all sigma points
  z_points = observation_model(sigma_points, map, z(i).id);

	% TODO: Compute the weights of the sigma points and compute zm.
	% zm is the recovered expected measurement mean from z_points.
	% It will be a 2x1 vector [expected_range; expected_bearing].

  % Compute weights:
  wm = [lambda/scale, repmat(1/(2*scale), 1, 2*n)];
  wc = wm;
  
  % Compute expected measurement
  cosines = sum(cos(z_points(2,:)).*wm);
  sines = sum(sin(z_points(2,:)).*wm);
  zm = [z_points(1,:) * wm'; atan2(sines, cosines)];
  
	% TODO: Compute the innovation covariance matrix S (2x2).
  delta_z = (z_points - zm);
  delta_z(2,:) = normalize_angle(delta_z(2,:));
  S = delta_z*diag(wc)*delta_z' + Q;

	% TODO: Compute sigma_x_z (which is equivalent to sigma times the Jacobian H transposed in EKF).
	% sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu  
  delta_mu = sigma_points - mu;
  delta_mu(3,:) = normalize_angle(delta_mu(3,:));
  sigma_x_z = delta_mu*diag(wc)*delta_z';

	% TODO: Compute the Kalman gain
  K = sigma_x_z*inv(S);

	% TODO: Update mu and sigma
  z_d = z_actual - zm;
  z_d(2) = normalize_angle(z_d(2));
 
  mu = mu + K*z_d;
  sigma = sigma - K*S*K';

	% TODO: Normalize the robot heading mu(3)
  mu(3) = normalize_angle(mu(3));

endfor
end
