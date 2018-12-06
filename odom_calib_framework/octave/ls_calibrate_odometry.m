% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = eye(3); 

  % TODO: initialize H and b of the linear system
  % Since H is linear combination of J'*Omega*J where
  % J is 9x3 and Omega is 3x3 it follows that H is 9x9
  H = zeros(9,9);
  
  % Similarly, H is 9x9  and X 9x1, which results in b being 9x1.
  b = zeros(9, 1);
  
  % Omega is the information matrix of 3x3
  omega = eye(3,3);
  
  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  for i=1:size(Z, 1)
    % compute the jacobian for this point
    J = jacobian(i,Z);
    
    % compute the error for this point
    e = error_function(i, X, Z);
    
    % compute H and b by summing each Hi and bi
    H = H + J'*omega*J;
    b = b + J'*omega'*e;
  endfor 
  
  % TODO: solve and update the solution
  IH = inv(H);
  delta_X = -IH*b;
  X = X + [delta_X(1:3)'; delta_X(4:6)'; delta_X(7:9)'];
  
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  
  % Ground truth odometry form scanmathing are the first 3 components of row i of Z
  u_gt = Z(i,1:3)';
  
  % Ground truth odometry measured by sensor are the last 3 components of row i of Z
  u = Z(i,4:6)';

  % Error computed as the diffrence between the ground truth and corrected measurements
  % through the calibration parameters X (see slide 26 in course 14).
  e = u_gt - X*u;
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  % As the derivation in slide 26 shows the Jacobian only containts the entries
  % which designate the measured odometry. 
  % Note that since we have a state vector X has 9 entries and the error function
  % is in R^3 the  resulting jacobian has a size of 9 x 3.
  J = -[Z(i,4) Z(i,5) Z(i,6) zeros(1,6); 
        zeros(1,3) Z(i,4) Z(i,5) Z(i,6) zeros(1,3); 
        zeros(1,6) Z(i,4) Z(i,5) Z(i,6)];
end
