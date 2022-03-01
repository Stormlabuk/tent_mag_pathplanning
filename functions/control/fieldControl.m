%function [X, U, err] = fieldControl(Ud, mu0, mu, K, numIt, step, k0)
function [X, U, err] = fieldControl(Ud, X, K, numIt, step, k0)


%% Find Optimal Initial Conditions for just the gradient
%X = magPosition_alphas(Ud(4:8)');

%Changing from alpha, theta, and rho to x,y,z
%Rot functions take arguments in degree
% x1 = rotz(theta*180/pi)*rotx(-alpha_final*180/pi)*[0;-rhoFinal;0];
% x2 = rotz(theta*180/pi)*rotx(alpha_final*180/pi)*[0;rhoFinal;0];
% x1 = rotz(theta*180/pi)*rotx(-alpha_final(1)*180/pi)*[0;-rhoFinal;0];
% x2 = rotz(theta*180/pi)*rotx(alpha_final(2)*180/pi)*[0;rhoFinal;0];

% if (int8(norm(mu_final(1:3))) ~= 1 || int8(norm(mu_final(4:6))) ~= 1 )
%     disp("Norm not 1");
%     norm(mu_final(1:3))
%     norm(mu_final(4:6))
% end
%     
% 
% m1 = mu*(mu_final(1:3)/norm(mu_final(1:3)));
% m2 = mu*(mu_final(4:6)/norm(mu_final(4:6)));
% 
% x = [x1; m1; x2; m2];

%x1 = [1; 0; 0];
%x2 = -x1;
% id = eye(6);

% dU_dm = [];
% for i = 1:6
%     m1 = mu*id(1:3, i);
%     m2 = mu*id(4:6, i);
%     X = [x1; m1; x2; m2];
% 
%     dU_dm = [dU_dm, field_2(mu0, X)];
% end
% 
% m = dU_dm\Ud(1:end - 2);
% ro1 = (mu/norm(m(1:3)))^(1/3);
% ro2 = (mu/norm(m(1:3)))^(1/3);
% m(1:3) = m(1:3)/norm(m(1:3))*mu;
% m(4:6) = m(4:6)/norm(m(4:6))*mu;
% x1 = ro1*x1;
% x2 = ro2*x2;
% x = [x1; m(1:3); x2; m(4:6)];

%% Constants
Id = eye(length(X));

%% Initial Conditions
%X(:, 1) = x;
%X(:, 1) = [0; -0.2707; 0.0549; -0.0; -583.3561; -775.1063; 0; 0.1253; 0.2462; 0.0; 583.3561; 775.1063];

%for i = 2:numIt
i = 2;
stop = 0;

while i < numIt %&& stop == 0
%for i = 2:numIt
    %% Task 1: Field Control
        % Output
        U(:, i - 1) = field(X(:, i - 1));

        % Error
        err(:, i - 1) = U(:, i - 1) - Ud;
        
        %Calculating norm err. Skip the first one as this cant be the
        %solution (still step 1 of grad descent)
        if(i ~= 2)
            errNorm(i-2) = norm(err(:,i-1));
        end

%         if norm(err(1:8, i - 1)) < 0.0001 || isreal(U(:, i - 1)) == 0
%             stop = 1;
%         end
        % Jacobian
        J = jacNum(X(:, i - 1), step);

        % Inversion
        Jp = J.'/(J*J.'); % jacobian pseudoinverse
        
        v = -K*Jp*err(:, i - 1);
        
    
    %% Task 2: Jacobian Rank Condition
    v2 = jacRank(X(:, i - 1), k0, step);
    
    %% Overall Control
    v = v + (Id - Jp*J)*v2;
    
    % Next Step
    X(:, i) = X(:, i - 1) + v;
    
    i = i + 1;
end

%Uncomment this if not running with Gradient Method
%Selecting one with lowest err
[M, I] = min(errNorm);

U = U(:,I);
err = err(1,I);
X = X(:,I);

end