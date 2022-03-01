%%Script to Test Mag Position Calculator with Random Gradients
clear all
close all
clc
%%

%Supressing warnings about Deficient Matrix Rank
warning('off','MATLAB:rankDeficientMatrix')

itt = 1000;
%Fail counters
failFPinv = 0;
failFPrange = 0;
failDbError = 0;
failDbAlpha = 0;
%Timing process
tic

dB_results = zeros(5, itt);
dB_request = zeros(5,itt);
for a = 1:itt
    
    if mod(a,100) == 0 || a == 1
        a
    end
    
    %Generating random norms for random fields and gradients
    B_norm = 0;%0.005*rand();
    dB_norm = 0.1*rand();
%     dB_norm = 0.1;
    
%     %Generating Random Fields (between -0.01 and 0.01)
%     B_des = -B_norm + (B_norm*2)*rand(1,3);
%     %Making sure norm of Random gradients is 0.01
%     B_des = B_norm*(B_des/norm(B_des));
%     
    %Generating Random Gradients (between -0.1 and 0.1)
    dB_des = -dB_norm + (dB_norm*2)*rand(1,5);
    %Making sure norm of Random gradients is 0.1
    dB_des = dB_norm*(dB_des/norm(dB_des));
    dB_request(:,a) = dB_des;
%     %Finding the focal point
%     [p0,errorFP] = focalPoint(B_des, dB_des);
%     failFPinv = failFPinv + errorFP;
    
    %Calculating parameters for gradient control
    %dB_des = [0.0,   0.0,  0.0,  0.1  0.0];
    [error_dB, error_alpha, dB_results(:, a),rho] = magPosition_alphas(dB_des);
    failDbError = failDbError + error_dB;
    failDbAlpha = failDbAlpha + error_alpha;

%     %seeing if rho is ever out of range
%     if (max(abs(p0)) + 0.15 > rho)
%         failFPrange = failFPrange +1;
%     end

end
%End timing
toc

errorPer = ((failDbError + failFPinv + failFPrange + failDbAlpha) / itt) * 100;

% bigError = error_ret > 0.005;
% fails = nnz(bigError);
% disp("Error over 0.005: " + fails + "\" + itt);
disp("Error as a %: " + errorPer + "%");

%plotting desired vs achieved for each gradient 
for a = 1:5

    figure(1)
        subplot(3, 2, a)
        plot(1:itt,dB_request(a,:), 'b')
        hold on 
        scatter(1:itt,dB_results(a,:),'r')
        xlabel('Sample Number');
        ylabel(strcat('$dBU_', num2str(a+3), '$'), 'Interpreter', 'latex');
        legend('Desired','Achieved')
        grid on;
end