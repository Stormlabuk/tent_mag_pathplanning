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

%U = [Bx,       By,        Bz,         dBxx,      dBxy,        dBxz,       dByy,       dByz,   mu1,    mu2]
Uc = [0.0;      0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
Ud = [[0.0;     0.00;      0.0022483;  0.0;        0.0;        0.1;        0.0;        0.0;    970.1;  970.1], ...
      [0.0;     0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.1;    970.1;  970.1], ...
      [0.00225; 0.0;       0.0;       -0.1;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
      [0.0;    -0.01;      0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
      [0.01;    0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
      [-0.001;  0.0036;   -0.000167;   0.0;       -0.1;        0.0;        0.0;        0.0;    970.1;  970.1], ...
      [-0.005; -0.00125;   0.0011;     0.1;        0.0;        0.0;       -0.1;        0.0;    970.1;  970.1], ...
      [-0.0024; 0.0017;   -0.0088;     0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1]];

%Constants for Field Finder
mu0 = 4*pi*1e-7;
mu = 970.1;
r_min = 0.1; %Minimum dist from magnet to origin
r_max = 1; %Maximum dist from magnet to origin
max_time = 3;

U_request = cat(2,Uc,Ud); %Creating one matrix for all the Fields we're requestion
[~, numFields] = size(Ud);

%Matrixes with the full Path for all requested fields. 
path_points = 6;
u_planFinal =  zeros([path_points*numFields, 6]);
X_planningFinal = zeros([path_points*numFields, 12]);
X_CSVFinal = zeros([path_points*numFields, 14]);
Uc_plotFinal = zeros([10,path_points*numFields]);
Ud_plotFinal = zeros([10,path_points*numFields]);
U_pathFinal = zeros([8, path_points*numFields]);


for count = 1:numFields

    Uc = U_request(:,count);
    Ud = U_request(:,count + 1);

    %Checking if moving to zero position
    if norm(Uc(1:8)) == 0
        Xc = [-0.35; -1.02; 0.19; 827.4953; 145.5150; -485.0500; 0.35; 1.02; 0.19; -827.4953; -145.5150; -485.0500];
        %[Xd,~,err_d,id]  = field_optimise.find(Ud,mu,r_min,r_max,100,1e-12);
        [Ud_found, Xd] = fieldSearchMBGP(Ud);
    
    elseif norm(Ud(1:8)) == 0
        Xd =[-0.35; -1.02; 0.19; 827.4953; 145.5150; -485.0500; 0.35; 1.02; 0.19; 827.4953; -145.5150; -485.0500];
        %[Xc,~,err_c,ic]  = field_optimise.find(Uc,mu,r_min,r_max,100,1e-12);
        [Uc_found, Xc] = fieldSearchMBGP(Uc);
    else
        [Uc_found, Xc] = fieldSearchMBGP(Uc);
        [Ud_found, Xd] = fieldSearchMBGP(Ud);
        %[Xc,~,err_c,ic]  = field_optimise.find(Uc,mu,r_min,r_max,100,1e-12);
        %[Xd,~,err_d,id]  = field_optimise.find(Ud,mu,r_min,r_max,100,1e-12);
        % robot_ik.displayState(Xd);
        
    end

     %% Creating a Linear path in polar space
    [X_planning, polarpath_1, polarpath_2] = pathCreate(Xc, Xd, path_points);


    %% Plotting Start (Current) and Desired Positions of both Magnets
    figure(1);
    subplot((numFields)/2 ,2, count)
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
    title(strcat('$Path_', num2str(count),' $'), 'Interpreter', 'latex', 'FontSize', 16)           
   
    %% Plotting Path
    hold on
    hf = plot3(polarpath_1(:,1), polarpath_1(:,2), polarpath_1(:,3), ':k', 'LineWidth',2);
    hold on
    hg = plot3(polarpath_2(:,1), polarpath_2(:,2), polarpath_2(:,3), ':g', 'LineWidth',2);
    
    %plotting the 0-axis
    hh = plot3(linspace(-1,1,100), zeros([1,100]), zeros([1,100]), 'k:');
    hi = plot3(zeros([1,100]), linspace(-1,1,100), zeros([1,100]), 'k:');
    hj = plot3(zeros([1,100]), zeros([1,100]), linspace(-1,1,100), 'k:');
    
    grid on;
    
    if count ==1
        lgd = legend([ha hb hc hd he hf hg], 'P1_{Current}','P2_{Current}','P1_{Desired}','P2_{Desired}', 'WorkSpace Center', 'Path 1', 'Path 2');
        lgd.FontSize = 14;
        sgtitle("Paths to be Taken", 'FontSize', 24)
    end
   
    
    %% Getting field at everypoint in path 
    U_path = zeros(10, path_points);
    
    for a = 1:path_points
        U_path(:,a) = field(X_planning(a,:));
    end
    
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

    u_planFinal((path_points*(count-1) + 1):(path_points*count), :) =  u_plan;
    X_planningFinal((path_points*(count-1) + 1):(path_points*count), :) = X_planning;
    X_CSVFinal((path_points*(count-1) + 1):(path_points*count), :) = X_CSV;
    
    Uc_plot = repmat(Uc, 1, path_points);
    Ud_plot = repmat(Ud, 1, path_points);
    Uc_plotFinal(:, (path_points*(count-1) + 1):(path_points*count)) = Uc_plot;
    Ud_plotFinal(:, (path_points*(count-1) + 1):(path_points*count)) = Ud_plot;
    U_pathFinal(:, (path_points*(count-1) + 1):(path_points*count)) = U_path;


end


%% Plotting Field at each point in path
U_final = zeros(10, path_points*count);

for a = 1:path_points*count
    U_final(:,a) = field(X_planningFinal(a,:));
end

figure();
for i = 1:8
    subplot(4, 2, i)
        plot(1:path_points*count, U_pathFinal(i, :)', '--', 'LineWidth', 1.0)
        hold on
        plot(1:path_points*count, Uc_plotFinal(i, :)', 'ro', 'LineWidth', 1.0)
        hold on
        plot(1:path_points*count, Ud_plotFinal(i, :)', 'go', 'LineWidth', 1.0)
        hold on
        plot(1:path_points*count, U_final(i,:), 'k--', 'LineWidth', 3.0)
        grid on
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)

        if i <= 3
            ylim([-0.01 0.01]);
        else
            ylim([-0.1 0.1]);
        end

        if i == 1
            legend('Path', 'Start', 'Desired', 'Path Followed', 'FontSize', 12)
        end        
end
sgtitle("Planned Field for Full Path", 'FontSize', 24)


%% Plotting Forces and torques
m1 = [0, -1, 0]%; * 1050423;
m2 = [0, 0, 1]%; * 1050423;

% Load Cell 1
m = m1;

S = zeros([6,8]); 
S(1,7:8) = [-m(3), m(2)];
S(2,6:8) = [m(3), 0, -m(1)];
S(3,6:8) = [-m(2), m(1), 0];
S(4,1:3) = [m(1), m(2), m(3)];
S(5,2:5) = [m(1), 0, m(2), m(3)];
S(6,1:5) = [-m(3), 0, m(1), -m(3), m(2)];

w1_des = S*Ud_plotFinal(1:8,:);
w1_plan = S*U_final(1:8,:);

%Load Cell 2
m = m2;

S = zeros([6,8]); 
S(1,7:8) = [-m(3), m(2)];
S(2,6:8) = [m(3), 0, -m(1)];
S(3,6:8) = [-m(2), m(1), 0];
S(4,1:3) = [m(1), m(2), m(3)];
S(5,2:5) = [m(1), 0, m(2), m(3)];
S(6,1:5) = [-m(3), 0, m(1), -m(3), m(2)];

w2_des = S*Ud_plotFinal(1:8,:);
w2_plan = S*U_final(1:8,:);

% Plotting
figure();
for i = 1:6
    subplot(3, 2, i)
        plot(1:path_points*count, w1_plan(i, :)', 'k', 'LineWidth', 1.0)
        hold on
        plot(1:path_points*count, w1_des(i, :)', 'r--', 'LineWidth', 2.0)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$w_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)

%         if i <= 3
%             ylim([-0.01 0.01]);
%         else
%             ylim([-0.1 0.1]);
%         end

        grid on;

        if i == 1
            legend('Wrench', 'Desired', 'FontSize', 12)
        end        
end
sgtitle("Planned Wrench w1 for Full Path", 'FontSize', 24)

figure();
for i = 1:6
    subplot(3, 2, i)
        plot(1:path_points*count, w2_plan(i, :)', 'k', 'LineWidth', 1.0)
        hold on
        plot(1:path_points*count, w2_des(i, :)', 'r--', 'LineWidth', 2.0)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$w_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)

%         if i <= 3
%             ylim([-0.01 0.01]);
%         else
%             ylim([-0.1 0.1]);
%         end

        grid on;

        if i == 1
            legend('Wrench', 'Desired', 'FontSize', 12)
        end        
end
sgtitle("Planned Wrench w2 for Full Path", 'FontSize', 24)