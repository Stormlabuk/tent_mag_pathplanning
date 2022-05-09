function U = field(x)
% Computes the field and differentials (U).

mu0 = 4*pi*1e-7; % air magnetic permeability
X = reshape(x, [6, 2]);

%% Outputs
B = zeros(3, 1);
dB = zeros(3, 3);

for i = 1:2
    r = X(1:3, i);
    ro = norm(r);
    m = X(4:6, i);
    mu = norm(m);

%     B = B + mu0*mu/(4*pi*(ro^3))*(3*(r)*(r).' - eye(3))*m;
%     
%     dB = dB + 3*mu0*mu/(4*pi*(ro^4))*((eye(3) - 5*(r)*(r).')*...
%         dot(r, m) + (r)*(m).' + ((r)*(m).').');
    
    B = B + mu0/(4*pi*(ro^3))*(3*(r/ro)*(r/ro).' - eye(3))*m;
    
    dB = dB + 3*mu0/(4*pi*(ro^4))*((eye(3) - 5*(r/ro)*(r/ro).')*...
        dot(r/ro, m) + (r/ro)*(m).' + ((r/ro)*(m).').');
end

U = [B; dB(1, 1); dB(1, 2); dB(1, 3); dB(2, 2); dB(2, 3); norm(X(4:6, 1));
    norm(X(4:6, 2))];

end