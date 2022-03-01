%function [failDbError, failAlpha, dB_final, rhoFinal] = magPosition_alphas(dB_des)    %Find alpha, rho and theta for desired Gradients 
%function [rhoFinal, theta, alpha_final, mu_final, dB_final] = magPosition_alphas(dB_des)    %Find alpha, rho and theta for desired Gradients 
function Xfinal = magPosition_alphas(dB_des)

    %Inputs: Desired Gradients      assumes desired field is 0
    B = [0,0,0];   
    fieldFinal = [B dB_des];
    %Output: Error between calculated and desired gradients

    %Error marker
    failDbError = 0;
    failAlpha = 0;
    
    % Finding theta
    theta  = -atan2(dB_des(3),dB_des(5));
    %theta = pi/2;

    %Finding M for different values of alpha
    alpha1_vals = -pi/2:0.05:pi/2;
    alpha2_vals = -pi/2:0.05:pi/2;

%     M_alphas = zeros(5, 6, length(alpha1_vals)*length(alpha2_vals));
    M_alphas = zeros(8, 6, length(alpha1_vals)*length(alpha2_vals));

    count = 1;
    for counta1 = 1:length(alpha1_vals)
        for counta2 = 1:length(alpha2_vals)
        
        % Creating Rotation Matrixes
        Rx1 = [1, 0, 0;
            0, cos(alpha1_vals(counta1)), -sin(alpha1_vals(counta1));
            0, sin(alpha1_vals(counta1)), cos(alpha1_vals(counta1))];
        
        Rx2 = [1, 0, 0;
            0, cos(alpha2_vals(counta2)), -sin(alpha2_vals(counta2));
            0, sin(alpha2_vals(counta2)), cos(alpha2_vals(counta2))];

        Rz = [cos(theta), -sin(theta), 0;
            sin(theta), cos(theta), 0; 
            0, 0, 1];
        
        r1 = Rz * Rx1 * [0; -1; 0];
        r2 = Rz * Rx2 * [0; 1; 0];

       

        %Finding coefs for each component of mu 
        M = [];
        for i = 1:6

            %Required magnet orientation
            mu_sub = zeros(6,1);
            mu_sub(i) = 1;

            Id = eye(3);

            %Dipole Model adapted with Rotation Matrixes
            
            B = ((3*r1*r1' - Id)*mu_sub(1:3) + (3*r2*r2' - Id)*mu_sub(4:6));
            jacob = (Id - 5*(r1*r1'))*(r1.'*mu_sub(1:3)) + mu_sub(1:3)*r1' + r1*mu_sub(1:3)' + (Id - 5*(r2*r2'))*(r2.'*mu_sub(4:6)) + mu_sub(4:6)*r2' + r2*mu_sub(4:6)';
            
%             eq1 = (Id - 5*Id(:, 2)*Id(2, :))*(Id(2, :)*mu_sub(1:3)) + mu_sub(1:3)*Id(:, 2).' + (mu_sub(1:3)*Id(:, 2).').';
%             eq2 = (Id - 5*Id(:, 2)*Id(2, :))*(Id(2, :)*mu_sub(4:6)) + mu_sub(4:6)*Id(:, 2).' + (mu_sub(4:6)*Id(:, 2).').';
% 
% 
%             jacob = Rz*(Rx1*eq1*Rx1.' + Rx2.'*eq2*Rx2)*Rz.';
            %jacob =  Rx*(Rz*eq*Rz')*Rx' + Rx'*(Rz*eq*Rz')*Rx;

            dB = [jacob(1, 1); jacob(1, 2); jacob(1, 3); jacob(2, 2); jacob(2, 3)];
            
            U = [B; dB];
            M = [M, U];

        end

               
        M_alphas(:,:, count) = M;
        M_norm(count) = norm(M_alphas(:,:, count));
        count = count + 1;
        
        end
    end


    %Finding rho for each value of alpha
    mu_not = (4*pi)*10^-7;
    EPM_mag  = 970.1;

    %rho = ((3*mu_not * EPM_mag)/(2*pi* norm(dB_des)) .* M_norm).^(1/4);
    rho = ((3*mu_not * EPM_mag)/(2*pi* norm(dB_des))).^(1/4);

    %Finding mu for each value of alpha
    mu_calc = zeros(6,length(alpha1_vals)*length(alpha2_vals));

    for count = 1:length(mu_calc)

        %mu_calc(:,count) = (2*pi*rho(count)^4)/(3*mu_not*EPM_mag) * (M_alphas(:,:, count)\ dB_des');
        %mu_calc(:,count) = M_alphas(:,:, count)\ dB_des';
        mu_calc(:,count) = (pinv(M_alphas(:,:, count))* fieldFinal');
        %normalising
%         mu_calc(1:3,count) = -mu_calc(1:3,count) / norm(mu_calc(1:3,count));
%         mu_calc(4:6,count) = -mu_calc(1:3,count);
        mu_calc(1:3,count) = mu_calc(1:3,count) / norm(mu_calc(1:3,count));
        mu_calc(4:6,count) = mu_calc(4:6,count) / norm(mu_calc(4:6,count));
        
        if isnan(mu_calc(4:6,count))
           mu_calc(4:6,count) = [0;0;0]; 
        end
    end

    %Substituing all values back to find gradients
    
    dB_calc = zeros(10,length(alpha1_vals)*length(alpha2_vals));
    count = 1;
    for counta1 = 1:length(alpha1_vals)
        for counta2 = 1:length(alpha1_vals)
            %Changing from alpha, theta, and rho to x,y,z
            %Rot functions take arguments in degree
            x1 = rotz(theta*180/pi)*rotx(-alpha1_vals(counta1)*180/pi)*[0;-rho;0];
            x2 = rotz(theta*180/pi)*rotx(alpha2_vals(counta2)*180/pi)*[0;rho;0];

            x = [x1; mu_calc(1:3,count)*EPM_mag; x2; mu_calc(4:6,count)*EPM_mag];

            X(:, count) = x;
            dB_calc(:, count) = field(X(:, count));
            count = count + 1;
        end
    end

%Calculating error for each value of alpha

for count = 1:length(mu_calc)
    errorB(:,count) = (dB_calc(1:3,count) - fieldFinal(1:3)');
    error_normB(count) = norm(errorB(:,count));
    
    errordB(:,count) = (dB_calc(4:8,count) - fieldFinal(4:8)');
    error_normdB(count) = norm(errordB(:,count));
    
    errorTot(:,count) = dB_calc(1:8,count) - fieldFinal';
    errorTot_norm(count) = norm(errorTot(:,count));
end


    [error_ret,I] = min(errorTot_norm);

%     %If smallest error is greater then 0.01 T/m
%     if min(norm(dB_calc(:,:,I) - dB_des)) > 0.01
%         %disp("Adjusting rho");
%         %Calculate Proportionality constant
%         k = norm(dB_calc(:,:,I)) * rho(I)^4;
%         %find new rho
%         rho(I) = (k/norm(dB_des))^(1/4);
%         %Find new dB
%         dB_calc(:,:,I) = ((3*mu_not*EPM_mag)/(2*pi*rho(I)^4)) * M_alphas(:,:, I) * mu_calc(:,I);
% 
%         %Recalculating Error for Adjusted rho
%         error(:,I) = abs(dB_calc(:,:,I) - dB_des');
%         error_ave(I) = mean(error(:,I));
% 
%         [error_ret,I] = min(error_ave);
%     end
    

    
    %Variables to Return
    Xfinal = X(:,I);
%     rhoFinal = rho;
%     dB_final = dB_calc(4:8,I);
%     try
%         alpha1_ind = floor(I/length(alpha1_vals)) + 1;
%         alpha2_ind = int8((I/length(alpha1_vals) - (alpha1_ind-1)) * length(alpha2_vals));
%         alpha1_final = alpha1_vals(alpha1_ind);
%         alpha2_final = alpha2_vals(alpha2_ind);
%         alpha_final = [alpha1_final,alpha2_final];
%     catch ME
%         disp("Problem");
%     end
%     mu_final = mu_calc(:,I);
%     
%     if error_ret > 0.001
%         failDbError = 1;
%     elseif alpha1_final > pi/3 || alpha2_final > pi/3
%         failAlpha = 1;
%     end
% 
%     failAlpha = 0;

end