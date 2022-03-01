%%Script to Field and Gradient calculator with Starting Position given
%by alpha, theta method
clear all
close all
clc
%%

%Supressing warnings about Deficient Matrix Rank
warning('off','MATLAB:rankDeficientMatrix')

itt = 10000;
%Fail counters
failFPinv = 0;
failFPrange = 0;
failDbError = 0;
failDbAlpha = 0;
%Timing process
tic

for a = 1:itt
    
    if mod(a,100) == 0 || a == 1
        a
    end
    
    %Generating random norms for random fields and gradients
    B_norm = 0.005*rand();
    dB_norm = 0.5*rand();
%     dB_norm = 0.1;
    
    %Generating Random Fields (between -0.01 and 0.01)
    B_des = -B_norm + (B_norm*2)*rand(1,3);
    %Making sure norm of Random gradients is 0.01
    B_des = B_norm*(B_des/norm(B_des));
    
    %Generating Random Gradients (between -0.1 and 0.1)
    dB_des = -dB_norm + (dB_norm*2)*rand(1,5);
    %Making sure norm of Random gradients is 0.1
    dB_des = dB_norm*(dB_des/norm(dB_des));

    
    %Calculating parameters for gradient control
    [error_dB, error_alpha, dB_final,rho] = magPosition(dB_des);
    failDbError = failDbError + error_dB;
    failDbAlpha = failDbAlpha + error_alpha;



end
%End timing
toc

errorPer = ((failDbError + failFPinv + failFPrange + failDbAlpha) / itt) * 100;

% bigError = error_ret > 0.005;
% fails = nnz(bigError);
% disp("Error over 0.005: " + fails + "\" + itt);
disp("Error as a %: " + errorPer + "%");
% 
% figure()
% plot(error_ret)
% xlabel('Sample Number');
% ylabel('Error (T/m)');


%First need to translate rho, alpha & theta to an [x,y,z] position

p = [-rho*sin(theta); -rho*cos(theta); rho*sin(alpha); rho*sin(theta); rho*cos(theta); rho*sin(alpha)];