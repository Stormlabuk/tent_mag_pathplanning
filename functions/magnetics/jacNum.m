function J = jacNum(x, step)

%% Constants
Id = eye(length(x));


%% Functions
U = @(y) field(y);

%% Outputs
J = zeros(length(U(x)), length(x));

for i = 1:length(x)
    X = x + step*Id(:, i);
    J(:, i) = (U(X) - U(x))/step;
end

end