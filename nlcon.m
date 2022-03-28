function [c, ceq] = nlcon(m)
    c = [];
    ceq(1) = norm([m(1), m(2), m(3)]) - 970.1;
    ceq(2) = norm([m(4), m(5), m(6)]) - 970.1;
end