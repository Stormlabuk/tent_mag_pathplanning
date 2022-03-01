function U = field_2(mu0, x)
% Computes the field and differentials (U).

X = reshape(x, [6, 2]);

%% Outputs
B = zeros(3, 1);
dB = zeros(3, 3);

for i = 1:2
    r = X(1:3, i);
    ro = norm(r);
    m = X(4:6, i);
    mu = norm(m);
    
    B = B + mu0/(4*pi*(ro^3))*(3*(r)*(r).' - eye(3))*m;
    
    dB = dB + 3*mu0/(4*pi*(ro^4))*((eye(3) - 5*(r)*(r).')*...
        dot(r, m) + (r)*(m).' + ((r)*(m).').');
end

U = [B; dB(1, 1); dB(1, 2); dB(1, 3); dB(2, 2); dB(2, 3)];

end