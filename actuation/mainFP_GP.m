%% Main Script to Calculate Magnet Position 
%Uses both (Gradient & Focal Point Planning) + Jacobian Descent
%Solution Method (Giovanni)

rmpath(genpath('functions'))
addpath(genpath('functions'))

clear
close all
clc

%Supressing warnings about Deficient Matrix Rank
warning('off','MATLAB:rankDeficientMatrix')

%Setting Parallel Pool Parameters
nw = 7;     %Number of Cores
% p = parpool('local', nw);

% Magnetics
mu = 970.1; % EPMs magnetization
mu0 = 4*pi*1e-7; % air magnetic permeability

% Control
step = 1e-5; % derivative difference
K = 1e-2; % primary task gain
numIt = 500; % max iterations to convergence
k0 = 1e-3; % secondary task gain


%% Starting Computation
%itt = 200;
itt = 1;

XFinal = zeros([12,itt]);
UFinal = zeros([10,itt]);
errFinal = zeros([itt,1]);


%Fail counters
nanErr = 0;
muErr = 0;
xbigErr = 0;
xsmallErr = 0;
mag2closeErr = 0;
totErr = 0;
failFPinv = 0;
fpOutErr = 0;

tic
for a = 1:itt
%parfor (a = 1:itt, nw)
    
    B_norm = 0.01*rand();
    dB_norm = 0.1*rand();
    %dB_norm = 0.1;
    
    %Generating Random Fields (between -0.01 and 0.01)
    B_des = -B_norm + (B_norm*2)*rand(3,1);
    %Making sure norm of Random gradients is 0.01
    B_des = B_norm*(B_des/norm(B_des));
    B_des = [0.0; 0.0; 0];
    
    %Generating Random Gradients (between -0.1 and 0.1)U
    dB_des = -dB_norm + (dB_norm*2)*rand(5,1);
    %Making sure norm of Random gradients is 0.1
    dB_des = dB_norm*(dB_des/norm(dB_des));
    dB_des = [0.1; 0.0; 0; 0.0; 0.0];
    
    U_request(:,a) = [B_des; dB_des];
    
    %% Desired Field
    Ud = [B_des; dB_des; mu; mu];
    
    
    %Calculating parameters for gradient control
    X = magPosition_alphas(Ud(4:8)');
    %Ucal(:,a) = field(X);
    Ucal = field(X);
    
    %Finding the focal point
    %Utest = [.001, 0, 0, 0, 0.1, 0, 0, 0];
    UFocal = [Ud(1:3) - Ucal(1:3) ;Ucal(4:8)];
    [p0,errorFP] = focalPoint(UFocal);     %Desired Field with found gradient
    failFPinv = failFPinv + errorFP;
    
    %Adding focal Point
    Xfp = X;
    Xfp(1:3) = Xfp(1:3) - p0;
    Xfp(7:9) = Xfp(7:9) - p0;
    
    %Caclulating Error
    UcalFP(:,a) = field(Xfp);
    Uerr(a) = norm(Ud- Ucal);
    UerrFP(a) = norm(Ud- UcalFP(:,a));
    
    %Checking if either of the magnets are too close to center  
    if norm(X(1:3)) < 0.1 ||  norm(Xfp(1:3)) < 0.1 || norm(X(7:9)) < 0.1 ||  norm(Xfp(7:9)) < 0.1    
        %fpOutErr = fpOutErr + 1;
        
        %Use Jacobian Descent
        if Uerr(a) < UerrFP(a)
           %If error without FP is smaller use that 
            [XFinal(:,a), UFinal(:,a), errFinal(a)] = fieldControl(Ud, X, K, numIt, step, k0);
        else
            [XFinal(:,a), UFinal(:,a), errFinal(a)] = fieldControl(Ud, Xfp, K, numIt, step, k0);
        end

        
    %Check if magnets are too close
    elseif norm(X(1:3) - X(7:9)) < 0.2 || norm(Xfp(1:3) - Xfp(7:9)) < 0.2 
        %mag2closeErr = mag2closeErr + 1;
        
        %Use Jacobian Descent
        if Uerr(a) < UerrFP(a)
           %If error without FP is smaller use that 
            [XFinal(:,a), UFinal(:,a), errFinal(a)] = fieldControl(Ud, X, K, numIt, step, k0);
        else
            [XFinal(:,a), UFinal(:,a), errFinal(a)] = fieldControl(Ud, Xfp, K, numIt, step, k0);
        end
        
        
    %If norm(mu) is not right send to Jac Descent
    elseif norm(Xfp(4:6)) < 969.1 || norm(Xfp(10:12)) < 969.1 || norm(Xfp(4:6)) > 971.1 || norm(Xfp(10:12)) > 971.1
        %Use Jacobian Descent
        if Uerr(a) < UerrFP(a)
           %If error without FP is smaller use that 
            [XFinal(:,a), UFinal(:,a), errFinal(a)] = fieldControl(Ud, X, K, numIt, step, k0);
        else
            [XFinal(:,a), UFinal(:,a), errFinal(a)] = fieldControl(Ud, Xfp, K, numIt, step, k0);
        end

    %Check Error between desired and calculated field
    elseif UerrFP(a) > 0.05 || Uerr(a) > 0.05
        %Use Jacobian Descent
        if Uerr(a) < UerrFP(a)
           %If error without FP is smaller use that 
            [XFinal(:,a), UFinal(:,a), errFinal(a)] = fieldControl(Ud, X, K, numIt, step, k0);
        else
            [XFinal(:,a), UFinal(:,a), errFinal(a)] = fieldControl(Ud, Xfp, K, numIt, step, k0);
        end
        

        
    else
    %Just use Gradient Method,Store result with least error
        if Uerr(a) < UerrFP(a)
           %If error without FP is smaller use that 
            XFinal(:,a) = X;
            UFinal(:,a) = Ucal;
            errFinal(a) = Uerr(a);
        else
            XFinal(:,a) = Xfp;
            UFinal(:,a) = UcalFP(:,a);
            errFinal(a) = UerrFP(a);
        end 
              
        
%         
    end



end
toc

%% Caclulating Errors
totErr = 0;

for i = 1:itt
    if isnan(XFinal(:,i)) == 1
        nanErr = nanErr + 1;
    elseif norm(XFinal(1:3,i) - XFinal(7:9,i)) < 0.1
        mag2closeErr = mag2closeErr + 1;
    elseif norm(XFinal(1:3,i)) < 0.1 || norm(XFinal(7:9,i)) < 0.1    
        fpOutErr = fpOutErr + 1;
    elseif norm(XFinal(4:6,i)) > 971.1 || norm(XFinal(10:12,i)) > 971.1
        muErr = muErr + 1;
    elseif norm(XFinal(4:6,i)) < 969.1 || norm(XFinal(10:12,i)) < 969.1 
        muErr = muErr + 1;
    end
    
end

totErr = nanErr + muErr + fpOutErr + mag2closeErr; 

disp('Error is: ')
disp(totErr/itt * 100)

count = 1;
for i = 1:itt
    if isnan(errFinal(i)) == 0
        errPlot(count) = errFinal(i);
        count = count + 1;
    else
        count = count + 1;
    end
    
end

%% Saving Results to MAT file
filename = '8DOF_remake_3.mat';
save(filename, 'itt', 'XFinal', 'U_request', 'UFinal', 'Uerr','UerrFP', 'errFinal');

%% Plotting desired vs achieved for each gradient 
for a = 1:8

    figure(1)
    %title("No FP Shift")
    subplot(4, 2, a)
    scatter(1:itt,U_request(a,:), 'b')
    hold on 
    scatter(1:itt,UFinal(a,:),'r')
    xlabel('Sample Number');
    ylabel(strcat('$U_', num2str(a), '$'), 'Interpreter', 'latex');
    legend('Desired','Achieved')
    grid on;
end

% for a = 1:8
% 
%     figure(2)
%     title("FP Shift")
%     subplot(4, 2, a)
%     plot(1:itt,U_request(a,:), 'b')
%     hold on 
%     scatter(1:itt,UcalFP(a,:),'r')
%     xlabel('Sample Number');
%     ylabel(strcat('$dBU_', num2str(a), '$'), 'Interpreter', 'latex');
%     legend('Desired','Achieved')
%     grid on;
% end
% 
figure(3)
title("Error")
plot(1:itt,Uerr, 'b');
hold on;
plot(1:itt, UerrFP, 'r');
hold on;
plot(1:itt, errFinal, 'g');
legend('No FP Shift', 'FP Shift', 'Final');

% FieldErrFP = norm((abs(U_request(1:3,:) - Ucal(1:3,:)))/U_request(1:3,:))*100;
count = 1;
for i = 1:itt
    if isnan(errFinal(i)) == 0
        errPlot(count) = errFinal(i);
        count = count + 1;
    end
    count = count + 1;
end
