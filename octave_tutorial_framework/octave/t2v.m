function [x] = t2v(M)
x= zeros(1,3);
x(1) = M(1, 3)/M(3,3);
x(2) = M(2, 3)/M(3,3);
x(3) = atan2(M(2,1), M(1,1));
end
