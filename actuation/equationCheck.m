syms theta alpha real
Rx = [1, 0, 0;
    0, cos(alpha), -sin(alpha);
    0, sin(alpha), cos(alpha)];
%Rx = eye(3)
Rz = [cos(theta), -sin(theta), 0;
    sin(theta), cos(theta), 0; 
    0, 0, 1];

mu = sym('mu', [3, 1]);

Id = eye(3);

eq = (Id - 5*Id(:, 2)*Id(2, :))*(Id(2, :)*mu) + mu*Id(:, 2).' + (mu*Id(:, 2).').';

%jacob = Rz*(Rx*eq*Rx.' + Rx.'*eq*Rx)*Rz.';
%jacob =  Rx*(Rz*eq*Rz')*Rx' + Rx'*(Rz*eq*Rz')*Rx;
jacob =  (Rz*eq*Rz');

dB = [jacob(1, 1); jacob(1, 2); jacob(1, 3); jacob(2, 2); jacob(2, 3)];
simplify(dB);

M = sym([]);
% for i = 1:5
%     c1 = coeffs(dB(i), mu(1), 'All');
%     if (size(c1) == 1)
%         c1 = 0;
%     end
%     c2 = coeffs(dB(i), mu(2), 'All');
%     if (size(c2) == 1)
%         c2 = 0;
%     end
%     c3 = coeffs(dB(i), mu(3), 'All');
%     if (size(c3) == 1)
%         c3 = 0;
%     end
% 
%     M = [M; [c1(1), c2(1), c3(1)]];
% end

for i = 1:3
    mu_subs = zeros([3 1]);
    mu_subs(i) = 1;
    
%     c = dB* pinv(mu);
    
    M = [M, subs(dB, mu, mu_subs)];
    
end

M
%simplify(M)

% syms a b real
% theta_num = atan(a, b);
% 
% M_alpha = subs(M, theta, theta_num)