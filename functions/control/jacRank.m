function q0 = jacRank(x0, k0, step)
% Maximum manipulability solution.
mu0 = 4*pi*1e-7;
%% Outputs
q0 = zeros(size(x0));

%% Constant
Id = eye(length(x0));

%% Functions
w = @(x) sqrt(det(jacNum(x, step)*(jacNum(x, step)).'));

for i = 1:length(x0)
    X = x0 + step*Id(:, i);
    
    q0(i) = k0*(w(X) - w(x0))/step;
end

end