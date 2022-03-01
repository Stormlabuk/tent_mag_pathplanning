clear
close all
clc

rmpath(genpath('functions'))
addpath(genpath('functions'))


r = sym('r', [3, 1]);
m = sym('m', [3, 1]);

B = 1/(norm(r))^5*(3*r*dot(r, m) - (norm(r))^2*m);

dB = jacobian(B, r);

R = sym('R', [3, 2]);
M = sym('M', [3, 2]);

B = subs(B, [r; m], [R(1:3, 1); M(1:3, 1)]) + subs(B, [r; m], [R(1:3, 2); M(1:3, 2)]);
dB = subs(dB, [r; m], [R(1:3, 1); M(1:3, 1)]) + subs(dB, [r; m], [R(1:3, 2); M(1:3, 2)]);

U = [B; dB(1, 1); dB(1, 2); dB(1, 3); dB(2, 2); dB(2, 3)];

r = reshape(R, [6, 1]);
m = reshape(M, [6, 1]);

dU = jacobian(U, [r; m]);

rank(dU)