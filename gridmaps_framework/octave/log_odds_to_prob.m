function p = log_odds_to_prob(l)
% Convert log odds l to the corresponding probability values p.
% l could be a scalar or a matrix.

% TODO: compute p.
[m, n] = size(l);
for i = 1:m
  for j = 1:n
    p(i,j) = 1 - 1/(1+exp(l(i,j)));
   endfor
endfor
end
