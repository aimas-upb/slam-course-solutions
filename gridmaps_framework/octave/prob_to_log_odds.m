function l=prob_to_log_odds(p)
% Convert proability values p to the corresponding log odds l.
% p could be a scalar or a matrix.

% TODO: compute l.
[m, n] = size(p);
for i = 1:m
  for j = 1:n
    l(i,j) = log(p(i,j)/(1-p(i,j)));
  endfor
endfor

end
