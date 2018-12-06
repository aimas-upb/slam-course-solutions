function [lp] = landmark_position(x, z)
   lp = v2t(x)*[z,1]';
end
