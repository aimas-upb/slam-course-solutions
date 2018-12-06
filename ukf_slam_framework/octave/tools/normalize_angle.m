function [phi] = normalize_angle(phi)
% Normalize phi to be between -pi and pi
% phi can also be a vector of angles

idx = find(phi>pi);

while(!isempty(idx))
	phi(idx) = phi(idx) - 2*pi;
	idx = find(phi>pi);
endwhile

idx = find(phi<-pi);

while(!isempty(idx))
	phi(idx) = phi(idx) + 2*pi;
	idx = find(phi<-pi);
endwhile

end
