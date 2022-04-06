%% Main to Plan the Path 
close all
clear all
%Adding folders and subfolders to path
addpath(genpath('actuation'));
addpath(genpath('functions'));
%addpath(genpath('/home/michael/Desktop/MATLAB Scripts/MagneticPlanner/robot_tools'));
addpath(genpath('C:\Users\elmbr\OneDrive - University of Leeds\MagneticPlanner'));
%%
%All Coordinates are in sensor frame

%U = [Bx,   By,     Bz,     dBxx,   dBxy,   dBxz,   dByy,       dByz,   mu1,    mu2]
Uc = [0.0;  0.005;    0.0;    0.0;    0.0;    0.0;     0.0;       0.0;  970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
Ud = [0.0;  0.0;    0.0;    0.1;    0.0;    0.0;     0.0;       0.0;    970.1;  970.1];

%Constants for Field Finder
mu0 = 4*pi*1e-7;
mu = 970.1;
r_min = 0.1; %Minimum dist from magnet to origin
r_max = 1; %Maximum dist from magnet to origin
max_time = 3;
    
%Checking if moving to zero position
if norm(Uc) == 0
    Xc = [-0.35; -1.02; 0.19; 827.4953; 145.5150; -485.0500; 0.35; 1.02; 0.19; -827.4953; -145.5150; -485.0500];
    %[Xd,~,err_d,id]  = field_optimise.find(Ud,mu,r_min,r_max,100,1e-12);
    [UFinal, Xd] = fieldSearchMBGP(Ud);

elseif norm(Ud) == 0
    Xd =[-0.35; -1.02; 0.19; 827.4953; 145.5150; -485.0500; 0.35; 1.02; 0.19; 827.4953; -145.5150; -485.0500];
    %[Xc,~,err_c,ic]  = field_optimise.find(Uc,mu,r_min,r_max,100,1e-12);
    [UFinal, Xc] = fieldSearchMBGP(Uc);
else
    [UFinal, Xc] = fieldSearchMBGP(Uc);
    [UFinal, Xd] = fieldSearchMBGP(Ud);
    %[Xc,~,err_c,ic]  = field_optimise.find(Uc,mu,r_min,r_max,100,1e-12);
    %[Xd,~,err_d,id]  = field_optimise.find(Ud,mu,r_min,r_max,100,1e-12);
    % robot_ik.displayState(Xd);
    
end


%% Plotting Start (Current) and Desired Positions of both Magnets
figure;
ha = plot3(Xc(1),Xc(2),Xc(3), 'b*', 'MarkerSize',12);
hold on;
hb = plot3(Xc(7),Xc(8),Xc(9), 'bo', 'MarkerSize',12);
hold on;
hc = plot3(Xd(1),Xd(2),Xd(3), 'r*', 'MarkerSize',12);
hold on;
hd = plot3(Xd(7),Xd(8),Xd(9), 'ro', 'MarkerSize',12);
hold on;
he = plot3(0,0,0,'.k','MarkerSize',40);

xlim([-1 1])
ylim([-1.2 1.2])
zlim([-1 1])

xlabel('x', 'FontSize',24);
ylabel('y', 'FontSize',24);
zlabel('z', 'FontSize',24);

grid on 

%% Creating a Linear path in polar space
path_points = 6;
[X_planning, polarpath_1, polarpath_2] = pathCreate(Xc, Xd, path_points);

%% Getting field at everypoint in path 
U_path = zeros(10, path_points);

for a = 1:path_points
    U_path(:,a) = field(X_planning(a,:));
end

%% Plotting Path
hold on
hf = plot3(polarpath_1(:,1), polarpath_1(:,2), polarpath_1(:,3), ':k', 'LineWidth',2);
hold on
hg = plot3(polarpath_2(:,1), polarpath_2(:,2), polarpath_2(:,3), ':g', 'LineWidth',2);

%plotting the 0-axis
hh = plot3(linspace(-1,1,100), zeros([1,100]), zeros([1,100]), 'k:');
hi = plot3(zeros([1,100]), linspace(-1,1,100), zeros([1,100]), 'k:');
hj = plot3(zeros([1,100]), zeros([1,100]), linspace(-1,1,100), 'k:');


lgd = legend([ha hb hc hd he hf hg], 'P1_{Current}','P2_{Current}','P1_{Desired}','P2_{Desired}', 'WorkSpace Center', 'Path 1', 'Path 2');
lgd.FontSize = 14;

%% Plotting Field at each point in path
Uc_plot = repmat(Uc, 1, path_points);
Ud_plot = repmat(Ud, 1, path_points);

figure();

for i = 1:8
    subplot(4, 2, i)
        plot(1:path_points, U_path(i, :), '--', 'LineWidth', 3.0)
        hold on
        plot(1:path_points, Uc_plot(i, :), 'go', 'LineWidth', 1.0)
        hold on
        plot(1:path_points, Ud_plot(i, :), 'ro', 'LineWidth', 1.0)
        grid on
        xlabel('Points in Path (s)')
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex')  

        if i == 1
            legend('Path', 'Start', 'Desired')
        end        
end
sgtitle("Before Optimisation", 'FontSize', 24)

%% Analitical Solver for Pre Defined Path 

%Creating a field path by linearly interpolating
U_path = zeros(8, path_points);
u_plan = zeros(6,path_points);

% Creating a Vector X to write to CSV
X_CSV = zeros(path_points,14);
origin = [1,0,0];

for i = 1:8
   U_path(i,:) = linspace(Uc(i), Ud(i), path_points); 
end

[u_plan, X_planning, X_CSV] = analyticalMewSolve(Xc, Xd, X_planning, U_path);

%% Plotting Field at each point in path
U_final = zeros(10, path_points);

for a = 1:path_points
    U_final(:,a) = field(X_planning(a,:));
end

figure();
for i = 1:8
    subplot(4, 2, i)
        plot(1:path_points, U_path(i, :), '--', 'LineWidth', 1.0)
        hold on
        plot(1:path_points, Uc_plot(i, :), 'go', 'LineWidth', 1.0)
        hold on
        plot(1:path_points, Ud_plot(i, :), 'ro', 'LineWidth', 1.0)
        hold on
        plot(1:path_points, U_final(i,:), 'k--', 'LineWidth', 3.0)
        grid on
        xlabel('Points in Path (s)')
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex')  

        if i == 1
            legend('Path', 'Start', 'Desired', 'Path Followed')
        end        
end
sgtitle("After Optimisation", 'FontSize', 24)
