function W = Skew(w)
% Compute the Skew matrix from w.

W = [0, -w(3), w(2);
    w(3), 0, -w(1);
    -w(2), w(1), 0];

end