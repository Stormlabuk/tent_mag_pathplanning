function x = rotateFrame(R, x0)
% Rotate the solution found in local frame.

x = zeros(size(x0));
for i = 1:size(x0, 2)
    for j = 1:2
        x(6*j - 5:6*j, i) = [ 
            R*x0(1:3, i);
            R*x0(4:6, i)];
    end
end

end