function R = Exp(w, l, s, ord, der)
% Compute the exponential map of SO(3), given the deflection w, the lenght
% l and the length variavle s, up to order ord, derived (or integrated) up
% der times.

R = zeros(3, 3);
W = Skew(w);

switch sign(der)
    case 0
        for k = 0:ord
            R = R + 1/factorial(k)*W^k/l^k*s^k; 
        end
    case 1
        for k = der:ord
            R = R + 1/factorial(k - der)*W^k/l^k*s^(k - der);
        end
    case -1
        for k = 0:ord
            R = R + 1/factorial(k - der)*W^k/l^k*s^(k - der);
        end
end

end