% Field control with multiple EPMs
clear
close all
clc

rmpath(genpath('functions'))
addpath(genpath('functions'))

%% Constants
% Magnetics
mu = 970.1; % EPMs magnetization
mu0 = 4*pi*1e-7; % air magnetic permeability

% Control
step = 1e-5; % derivative difference
K = 1e-2; % primary task gain
numIt = 500; % max iterations to convergence
k0 = 1e-3; % secondary task gain

%%Testing for Random Fields and Gradients
testitt = 1;

% %Setting Parallel Pool Parameters
% nw = 8;
% p = parpool('local', nw);

tic
%parfor (a = 1:testitt, nw)
for a = 1:testitt
    
    B_norm = 0.005*rand();
    dB_norm = 0.1*rand();
    
    %Generating Random Fields (between -0.01 and 0.01)
    B_des = -B_norm + (B_norm*2)*rand(3,1);
    %Making sure norm of Random gradients is 0.01
    B_des = B_norm*(B_des/norm(B_des));
    
    %Generating Random Gradients (between -0.1 and 0.1)
    dB_des = -dB_norm + (dB_norm*2)*rand(5,1);
    %Making sure norm of Random gradients is 0.1
    dB_des = dB_norm*(dB_des/norm(dB_des));
    U_request(:,a) = [B_des; dB_des];
    
    %% Desired Field
    Ud = [B_des; dB_des; mu; mu];
    
    Ud = [0.001*[0; 0; 0]; 
        [-0.025; 0.0; 0.0; 0.025; 0];
        mu;
        mu];


    %% Control
    [X, U, err] = fieldControl(Ud, mu0, mu, K, numIt, step, k0);
    
    U_recieved(:,a) = U(1:8,end);
    
    normErr(a) = norm(U(1:8,end) -  U_request(:,a));
    Xfinal(:,a) = X(:,end);
end
toc
%% Saving Results to MAT file
filename = 'results.mat';
save(filename, 'testitt', 'Xfinal', 'U_recieved', 'U_request', 'normErr');
%% Plot
close all
t = 1:size(U, 2);

% Field and Error
for i = 1:8
    figure(1)
    subplot(4, 2, i)
        plot(t, U(i, :), '--', 'LineWidth', 3.0)
        hold on
        plot(t, repmat(Ud(i), [1, length(t)]), 'LineWidth', 3.0)
        
        grid on
        xlabel('time (s)')
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex')  
        if i == 1
            legend('Real', 'Desired')
        end
            
    figure(2)
    subplot(4, 2, i)
        plot(t, err(i, :), 'LineWidth', 3.0)
        
        grid on
        xlabel('time (s)')
        ylabel(strcat('$\tilde{U}_', num2str(i),'$'), 'Interpreter', 'latex')     
        
end

% Magnetization Error
figure(3)
subplot(2, 1, 1)
    plot(t, err(end - 1, :), 'LineWidth', 3.0)
        grid on
        xlabel('time (s)')
        ylabel(strcat('$\tilde{m}_1 $'), 'Interpreter', 'latex')
        
subplot(2, 1, 2)
    plot(t, err(end, :), 'LineWidth', 3.0)
        grid on
        xlabel('time (s)')
        ylabel(strcat('$\tilde{m}_2 $'), 'Interpreter', 'latex')
%         
% %% Final Results
% 
% 
% %plotting desired vs achieved for each gradient 
% for a = 1:8
% 
%     figure(1)
%         subplot(4, 2, a)
%         plot(1:testitt,U_request(a,:), 'b')
%         hold on 
%         scatter(1:testitt,U_recieved(a,:),'r')
%         xlabel('Sample Number');
%         ylabel(strcat('$dBU_', num2str(a), '$'), 'Interpreter', 'latex');
%         legend('Desired','Achieved')
%         grid on;
% end
% 
% figure(2)
% plot(normErr)
% grid on;

% Xdes = X(:, end);
% Xdes(4:6) = Xdes(4:6)/norm(Xdes(4:6))*mu;
% Xdes(10:12) = Xdes(10:12)/norm(Xdes(10:12))*mu; 
% 
% finalU =  field(Xdes);
% if isreal(finalU)
%     ERR = Ud - field(Xdes);
% else
%     finalU = real(finalU);
%     Xdes = real(Xdes)
% end
% 
% % Result
% ERR = Ud - field(Xdes);
% 
% disp("Desired Field & Gradient");
% disp(Ud)
% disp("Closest Achievable Field & Gradient");
% disp(finalU)
% disp("Position of Magnets:");
% disp(Xdes)

