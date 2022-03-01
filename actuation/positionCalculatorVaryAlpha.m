%% ----Main for Magnetic Gradient Solver (8 DoF Actuation)---- %% 
%% -- Each Magnet Allowed to have a diff alpha -- %%
clear all
close all
% clc

%% Enter Fields and Gradient
B = [0,0,0];            % Bx        By        Bz
dB_des = [0.0,   0.0,  0.0,  0.0  0.1];      % dBx/dx    dBx,dy    dBx,dz    dBy,dy    dBy,dz  

%% Find Focal Point
%Create Function to Find focal point
p0 = focalPoint(B,dB_des);

%% Find parameters for magnet positions

% Finding theta
theta  = -atan2(dB_des(3),dB_des(5));
%theta = pi/2;

% Creating Rotation Matrixes
syms alpha1 alpha2 real
Rx1 = [1, 0, 0;
    0, cos(alpha1), -sin(alpha1);
    0, sin(alpha1), cos(alpha1)];

Rx2 = [1, 0, 0;
    0, cos(alpha2), -sin(alpha2);
    0, sin(alpha2), cos(alpha2)];

Rz = [cos(theta), -sin(theta), 0;
    sin(theta), cos(theta), 0; 
    0, 0, 1];

%Required magnet orientation
mu = sym('mu', [6, 1]);

Id = eye(3);

%Dipole Model adapted with Rotation Matrixes
eq1 = (Id - 5*Id(:, 2)*Id(2, :))*(Id(2, :)*mu(1:3)) + mu(1:3)*Id(:, 2).' + (mu(1:3)*Id(:, 2).').';
eq2 = (Id - 5*Id(:, 2)*Id(2, :))*(Id(2, :)*mu(4:6)) + mu(4:6)*Id(:, 2).' + (mu(4:6)*Id(:, 2).').';


jacob = Rz*(Rx1*eq1*Rx1.' + Rx2.'*eq2*Rx2)*Rz.';
%jacob =  Rx*(Rz*eq*Rz')*Rx' + Rx'*(Rz*eq*Rz')*Rx;

dB = [jacob(1, 1); jacob(1, 2); jacob(1, 3); jacob(2, 2); jacob(2, 3)];
simplify(dB);

%Finding coefs for each component of mu 
M = [];
for i = 1:6

    mu_sub = zeros(6,1);
    mu_sub(i) = 1;

    dB_subs = subs(dB, mu, mu_sub);
    
    
    M = [M, dB_subs];
   
end


%Finding M for different values of alpha
alpha1_vals = 0:0.05:pi/2;%;3*pi/4;
alpha2_vals = 0:0.05:pi/2;
% alpha_vals = 3*pi/4;error_ave

M_alphas = zeros(5, 6, length(alpha1_vals)*length(alpha2_vals));
M_norm = zeros(1,length(alpha1_vals)*length(alpha2_vals));
count = 1;
for counta1 = 1:length(alpha1_vals)
    %count = (counta1-1) * length(alpha1_vals) + 1;
    for counta2 = 1:length(alpha1_vals)
        M_alphas(:,:, count) = double(subs(M, [alpha1,alpha2], [alpha1_vals(counta1), alpha2_vals(counta2)]));
        M_norm(count) = norm(M_alphas(:,:, count));
        count = count + 1;
    end
end


%Finding rho for each value of alpha
mu_not = (4*pi)*10^-7;
EPM_mag  = 970.1;

rho = ((3*mu_not * EPM_mag)/(2*pi* norm(dB_des)) .* M_norm).^(1/4);


%Finding mu for each value of alpha
mu_calc = zeros(6,length(alpha1_vals) * length(alpha2_vals));

for count = 1:length(mu_calc)

    mu_calc(:,count) = (2*pi*rho(count)^4)/(3*mu_not*EPM_mag) * (M_alphas(:,:, count)\ dB_des');
    %normalising
    mu_calc(1:3,count) = mu_calc(1:3,count) / norm(mu_calc(1:3,count));
    mu_calc(4:6,count) = mu_calc(4:6,count) / norm(mu_calc(4:6,count));
end

%Substituing all values back to find gradients
dB_calc = zeros(5,1,length(mu_calc));

for count = 1:length(mu_calc)
    dB_calc(:,:,count) = ((3*mu_not*EPM_mag)/(2*pi*rho(count)^4)) * M_alphas(:,:, count) * mu_calc(:,count);
end

%Calculating error for each value of alpha

for count = 1:length(mu_calc)
    error(:,count) = (dB_calc(:,:,count) - dB_des');
    error_ave(count) = norm(error(:,count));
end


[M,I] = min(error_ave);

% % If smallest error is greater then 0.01 T/m
% if min(norm(dB_calc(:,:,I) - dB_des)) > 0.01
%     disp("Adjusting rho");
%     %Calculate Proportionality constant
%     k = norm(dB_calc(:,:,I)) * rho(I)^4;
%     %find new rho
%     rho(I) = (k/norm(dB_des))^(1/4);
%     %Find new dB
%     dB_calc(:,:,I) = ((3*mu_not*EPM_mag)/(2*pi*rho(I)^4)) * M_alphas(:,:, I) * mu_calc(:,I);
%     
%     %Recalculating Error for Adjusted rho
%     error(:,I) = abs(dB_calc(:,:,I) - dB_des');
%     error_ave(I) = mean(error(:,I));
% 
%     [M,I] = min(error_ave);
% end
   
%alpha indexes
alpha1_ind = floor(I/length(alpha1_vals)) + 1;
alpha2_ind = (I/length(alpha1_vals) - (alpha1_ind-1)) * length(alpha2_vals) + 1;

disp("Focal Point");
disp(p0)
disp("Desired Gradient");
disp(dB_des)
disp("Closest Achievable Gradient");
disp(dB_calc(:,:,I)')
fprintf("Theta = " + theta + " rho = " + rho(I) + "\n");
fprintf(" alpha1 = " + (180*alpha1_vals(alpha1_ind))/pi + " alpha2 = " + (180*alpha2_vals(alpha2_ind))/pi + "\n")
disp("mu");
disp(mu_calc(:,I))

figure;
plot(error_ave)
xlabel('Alpha (degree)')
ylabel('Error')


