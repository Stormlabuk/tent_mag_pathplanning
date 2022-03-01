%% Trying new Method for Calculating alpha, theta and rho
clc 
close all
clear all

rmpath(genpath('functions'))
addpath(genpath('functions'))
%% Enter Fields and Gradient
B = [0,0,0];            % Bx        By        Bz
dB_des = [0.1,   0.0,  0.0,  0.0  0.0];      % dBx/dx    dBx,dy    dBx,dz    dBy,dy    dBy,dz  

fieldFinal = [B dB_des];

%% Find Focal Point
%Create Function to Find focal point
%p0 = focalPoint(B,dB_des);
%% Creating Variables

% Finding theta
theta  = -atan2(dB_des(3),dB_des(5));
%syms theta real
lambda_dB = 1;
lambda_B = 1;

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

id = eye(3);

mu1 = sym('mu1', [3, 1], 'real');
mu2 = sym('mu2', [3, 1], 'real');

r1 = Rz * Rx1 * [0; -1; 0];
r2 = Rz * Rx2 * [0; 1; 0];

jacob = lambda_dB * ((id - 5*(r1*r1'))*(r1.'*mu1) + mu1*r1' + r1*mu1' + (id - 5*(r2*r2'))*(r2.'*mu2) + mu2*r2' + r2*mu2');

B = lambda_B * ((3*r1*r1' - id)*mu1 + (3*r2*r2' - id)*mu2);
dB = [jacob(1, 1); jacob(1, 2); jacob(1, 3); jacob(2, 2); jacob(2, 3)];

U = [B; dB];

M = sym([]);

for i = 1:6
    mu_subs = zeros([3 1]);
    
    if i < 4
        mu_subs(i) = 1;
%         temp = subs(dB, mu1, mu_subs);
%         M = [M, subs(temp, mu2, zeros([3 1]))];
        
        temp = subs(U, mu1, mu_subs);
        M = [M, subs(temp, mu2, zeros([3 1]))];
    else
        mu_subs(i-3) = 1;
%         temp = subs(dB, mu2, mu_subs);
%         M = [M, subs(temp, mu1, zeros([3 1]))];
        
        temp = subs(U, mu2, mu_subs);
        M = [M, subs(temp, mu1, zeros([3 1]))];
    end
  	
    
end

%Finding M for different values of alpha
alpha1_vals = 0:0.1:pi/2;%;3*pi/4;
alpha2_vals = 0:0.1:pi/2;

% alpha_vals = 3*pi/4;error_ave

% M_alphas = zeros(5, 6, length(alpha1_vals));
M_alphas = zeros(8, 6, length(alpha1_vals));
M_norm = zeros(1,length(alpha1_vals)*length(alpha2_vals));
count = 1;
for counta1 = 1:length(alpha1_vals)
    %count = (counta1-1) * length(alpha1_vals) + 1;
    for counta2 = 1:length(alpha1_vals)
        M_alphas(:,:, count) = double(subs(M, {alpha1 alpha2}, {alpha1_vals(counta1) alpha2_vals(counta2)}));
        M_norm(count) = norm(M_alphas(:,:, count));
        count = count + 1;
    end
end

% 
% for count = 1:length(alpha1_vals)
%     M_alphas(:,:, count) = double(subs(M, {alpha1 alpha2}, {alpha1_vals(count) alpha_vals1(count)}));
%     M_norm(count) = norm(M_alphas(:,:, count));
% end


%Finding rho for each value of alpha
mu_not = (4*pi)*10^-7;
EPM_mag  = 970.1;

% rho = ((3*mu_not * EPM_mag)/(2*pi* norm(dB_des)) .* M_norm).^(1/4);
rho = ((3*mu_not * EPM_mag)/(2*pi* norm(dB_des))).^(1/4);



%Finding mu for each value of alpha
mu_calc = zeros(6,(length(alpha1_vals)*length(alpha2_vals)));

for count = 1:length(mu_calc)

%     mu_calc(:,count) = (2*pi*rho(count)^4)/(3*mu_not*EPM_mag) * (pinv(M_alphas(:,:, count))* dB_des');
%     mu_calc(:,count) = (pinv(M_alphas(:,:, count))* dB_des');
    mu_calc(:,count) = (pinv(M_alphas(:,:, count))* fieldFinal');
    %normalising
    mu_calc(1:3,count) = mu_calc(1:3,count) / norm(mu_calc(1:3,count));
    mu_calc(4:6,count) = mu_calc(4:6,count) / norm(mu_calc(4:6,count));
   

end

%Substituing all values back to find gradients
%dB_calc = zeros(5,1,length(mu_calc));

% for count = 1:length(mu_calc)
%     %Changing from alpha, theta, and rho to x,y,z
%     %Rot functions take arguments in degree
%     x1 = rotz(theta*180/pi)*rotx(-alpha_vals(count)*180/pi)*[0;-rho;0];
%     x2 = rotz(theta*180/pi)*rotx(alpha_vals(count)*180/pi)*[0;rho;0];
%     %mu_calc(4:6,count) = -mu_calc(1:3,count);
%     x = [x1; -mu_calc(1:3,count)*EPM_mag; x2; mu_calc(1:3,count)*EPM_mag];
%     
%     X(:, 1) = x;
%     U(:, count) = field(X(:, count));
% 
%     %dB_calc(:,:,count) = ((3*mu_not*EPM_mag)/(2*pi*rho(count)^4)) * (3*mu_not*EPM_mag/(4*pi*rho^4)) * M_alphas(:,:, count) * mu_calc(:,count);
%     %mu_calc(4:6,count) = -mu_calc(1:3,count);
% end

dB_calc = zeros(10,length(alpha1_vals)*length(alpha2_vals));
count = 1;
for counta1 = 1:length(alpha1_vals)
    %count = (counta1-1) * length(alpha1_vals) + 1;
    for counta2 = 1:length(alpha1_vals)
        %Changing from alpha, theta, and rho to x,y,z
        %Rot functions take arguments in degree
        x1 = rotz(theta*180/pi)*rotx(-alpha1_vals(counta1)*180/pi)*[0;-rho;0];
        x2 = rotz(theta*180/pi)*rotx(alpha2_vals(counta2)*180/pi)*[0;rho;0];
        %mu_calc(4:6,count) = -mu_calc(1:3,count);
        x = [x1; mu_calc(1:3,count)*EPM_mag; x2; mu_calc(4:6,count)*EPM_mag];

        X(:, count) = x;
        dB_calc(:, count) = field(X(:, count));
        count = count + 1;
    end
end

%Calculating error for each value of alpha

for count = 1:length(mu_calc)
    error(:,count) = (dB_calc(1:8,count) - fieldFinal');
    error_ave(count) = norm(error(:,count));
end


[M,I] = min(error_ave);

% %If smallest error is greater then 0.01 T/m
% if min(norm(dB_calc(4:8,I) - dB_des)) > 0.01
%     disp("Adjusting rho");
%     %Calculate Proportionality constant
%     k = norm(dB_calc(4:8,I)) * rho^4;
%     %find new rho
%     rho = (k/norm(dB_des))^(1/4);
%     %Find new dB
%     dB_calc(4:8,I) = ((3*mu_not*EPM_mag)/(2*pi*rho^4)) * M_alphas(:,:, I) * mu_calc(:,I);
%     
%     %Recalculating Error for Adjusted rho
%     error(:,I) = abs(dB_calc(:,:,I) - dB_des');
%     error_ave(I) = mean(error(:,I));
% 
%     [M,I] = min(error_ave);
% end
%    
%alpha indexes
alpha1_ind = floor(I/length(alpha1_vals)) + 1;
alpha2_ind = (I/length(alpha1_vals) - (alpha1_ind-1)) * length(alpha2_vals);

% disp("Focal Point");
% disp(p0)
disp("Desired Gradient");
disp(fieldFinal)
disp("Closest Achievable Gradient");
disp(dB_calc(1:8,I)')
fprintf("Theta = " + theta + " rho = " + rho + "\n");
fprintf(" alpha1 = " + (180*alpha1_vals(alpha1_ind))/pi + " alpha2 = " + (180*alpha2_vals(alpha2_ind))/pi + "\n")
disp("mu");
disp(mu_calc(:,I))

figure;
plot(error_ave)
xlabel('Alpha (degree)')
ylabel('Error')

