function [Ur, R] = rotateField(U)
% Compute the rotated field (Ur) to align with the y-axis and the rotation
% needed (R).

% Compute the rotation between Br and Bd
R = Vec2Rot([0; 1; 0], U(1:3)); 

% Extract the differential
dB = zeros(3);

k = 1;
for i = 1:2
    for j = i:3
        dB(i, j) = U(3 + k); 
        k = k + 1;
    end
end

dB = 0.5*dB + 0.5*dB.';
dB(3, 3) = -dB(1, 1) - dB(2, 2);

% Rotate the differential
dB = R*dB*R.';

% Find the new Field
Ur = U;
Ur(1:3) = R*U(1:3);

k = 1;
for i = 1:2
    for j = i:3
        Ur(3 + k) = dB(i, j);  
        k = k + 1;
    end
end


end