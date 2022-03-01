function [failDbError, failAlpha, dB_final, rhoFinal] = magPosition(dB_des)    %Find alpha, rho and theta for desired Gradients 
%function [rhoFinal, theta, alpha_final, mu_final, dB_final] = magPosition(dB_des)    %Find alpha, rho and theta for desired Gradients 
 
    %Inputs: Desired Gradients
    %Output: Error between calculated and desired gradients

    %Error marker
    failDbError = 0;
    failAlpha = 0;
    
    % Finding theta
    theta  = -atan2(dB_des(3),dB_des(5));
    %theta = pi/2;

    %Finding M for different values of alpha
    alpha_vals = 0:0.01:pi/2;
   

    M_alphas = zeros(5, 6, length(alpha_vals));

    for count = 1:length(alpha_vals)
        
        % Creating Rotation Matrixes
        Rx = [1, 0, 0;
            0, cos(alpha_vals(count)), -sin(alpha_vals(count));
            0, sin(alpha_vals(count)), cos(alpha_vals(count))];

        Rz = [cos(theta), -sin(theta), 0;
            sin(theta), cos(theta), 0; 
            0, 0, 1];

       

        %Finding coefs for each component of mu 
        M = [];
        for i = 1:6

            %Required magnet orientation
            mu_sub = zeros(6,1);
            mu_sub(i) = 1;

            Id = eye(3);

            %Dipole Model adapted with Rotation Matrixes
            eq1 = (Id - 5*Id(:, 2)*Id(2, :))*(Id(2, :)*mu_sub(1:3)) + mu_sub(1:3)*Id(:, 2).' + (mu_sub(1:3)*Id(:, 2).').';
            eq2 = (Id - 5*Id(:, 2)*Id(2, :))*(Id(2, :)*mu_sub(4:6)) + mu_sub(4:6)*Id(:, 2).' + (mu_sub(4:6)*Id(:, 2).').';


            jacob = Rz*(Rx*eq1*Rx.' + Rx.'*eq2*Rx)*Rz.';
            %jacob =  Rx*(Rz*eq*Rz')*Rx' + Rx'*(Rz*eq*Rz')*Rx;

            dB = [jacob(1, 1); jacob(1, 2); jacob(1, 3); jacob(2, 2); jacob(2, 3)];
          
            M = [M, dB];

        end

               
        M_alphas(:,:, count) = M;
        M_norm(count) = norm(M_alphas(:,:, count));
    end


    %Finding rho for each value of alpha
    mu_not = (4*pi)*10^-7;
    EPM_mag  = 970.1;

    rho = ((3*mu_not * EPM_mag)/(2*pi* norm(dB_des)) .* M_norm).^(1/4);


    %Finding mu for each value of alpha
    mu_calc = zeros(6,length(alpha_vals));

    for count = 1:length(alpha_vals)

        mu_calc(:,count) = (2*pi*rho(count)^4)/(3*mu_not*EPM_mag) * (M_alphas(:,:, count)\ dB_des');
        %normalising
        mu_calc(1:3,count) = mu_calc(1:3,count) / norm(mu_calc(1:3,count));
        mu_calc(4:6,count) = mu_calc(4:6,count) / norm(mu_calc(4:6,count));
    end

    %Substituing all values back to find gradients
    dB_calc = zeros(5,1,length(alpha_vals));

    for count = 1:length(alpha_vals)
        dB_calc(:,:,count) = ((3*mu_not*EPM_mag)/(2*pi*rho(count)^4)) * M_alphas(:,:, count) * mu_calc(:,count);
    end

    %Calculating error for each value of alpha

    for count = 1:length(alpha_vals)
        error(:,count) = abs(dB_calc(:,:,count) - dB_des');
        error_ave(count) = mean(error(:,count));
    end


    [error_ret,I] = min(error_ave);

    %If smallest error is greater then 0.01 T/m
    if min(norm(dB_calc(:,:,I) - dB_des)) > 0.01
        %disp("Adjusting rho");
        %Calculate Proportionality constant
        k = norm(dB_calc(:,:,I)) * rho(I)^4;
        %find new rho
        rho(I) = (k/norm(dB_des))^(1/4);
        %Find new dB
        dB_calc(:,:,I) = ((3*mu_not*EPM_mag)/(2*pi*rho(I)^4)) * M_alphas(:,:, I) * mu_calc(:,I);

        %Recalculating Error for Adjusted rho
        error(:,I) = abs(dB_calc(:,:,I) - dB_des');
        error_ave(I) = mean(error(:,I));

        [error_ret,I] = min(error_ave);
    end
    
    if error_ret > 0.005
        failDbError = 1;
    elseif alpha_vals(I) > pi/2.5
        failAlpha = 1;
    end
    
    %Variables to Return
    rhoFinal = rho(I);
    dB_final = dB_calc(:,:,I);
    alpha_final = alpha_vals(I);
    mu_final = mu_calc(:,I);


end