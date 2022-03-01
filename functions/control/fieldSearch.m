function X = fieldSearch(U, EPS)
% Seach for optimal solution

err = 1e10; % error
e1 = [1; 0; 0]; 
mu0 = 4*pi*1e-7;

id = eye(6);
M = zeros(6, 1);
R = zeros(6, 1);

for gamma1 = -pi:pi/100:pi
    for gamma2 = -pi:pi/100:pi
       for gamma3 = -pi:pi/100:pi
          r1 = rotx(gamma1)*roty(gamma2)*roty(gamma3)*e1; 
          r2 = -r1;

          % Compute the Jacobian w.r.t. mu
          dU_dm = [];
          for i = 1:6
              m1 = id(1:3, i);
              m2 = id(4:6, i);
              X = [r1; m1; r2; m2];
              dU_dm = [dU_dm, field_2(mu0, X)];
          end

          m = dU_dm\U;
          if (norm(dU_dm*m) < err)
              M = m;
              R = [r1; r2];
              if (norm(dU_dm*m) < EPS)
                break;
              end
          end
       end
    end
end

X = [R(1:3); M(1:3); R(4:6); M(4:6)];

end