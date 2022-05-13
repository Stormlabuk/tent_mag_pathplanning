%% Main to Plan the Path 
close all
clear all
%Adding folders and subfolders to path
addpath(genpath('actuation'));
addpath(genpath('functions'));
%addpath(genpath('/home/michael/Desktop/MATLAB Scripts/MagneticPlanner/robot_tools'));
addpath(genpath('C:\Users\elmbr\OneDrive - University of Leeds\Publications\MagneticPlanner'));
%%
%All Coordinates are in sensor frame

%Enter Desired End Effector Positions
Xdes = [[-0.35;     -1.02;    0.19;   827.4953;   145.5150;  -485.0500;  0.35;    1.02;     0.19;   -827.4953;  -145.5150;  -485.0500], ...
         [0.1909;   -0.1909;  0.00;   685.9643;   685.9643;   0.0;      -0.1909;  0.1909;   0.00;   -685.9643;  -685.9643;   0.0], ...         %f2y
         [0.35;      0;       0.0;    970.1;      0.0;        0.0;      -0.35;    0.0;      0.00;    970.1;      0.0;        0.0], ...         %t1y
         [0.1909;    0;      -0.1909; -686.0814;   0.0;      -685.8472; -0.1909;  0;        0.1909;  685.8472;   0.0;        686.0613], ...    %f1z
         [0.27;      0;       0.0;    0.;        -0.0;        970.1;    -0.27;    0;        0.00;    0;          0.0;       -970.1], ...       %f1x
         [0.27;      0;       0.0;    0.;        -970.1;      0.0;      -0.27;    0;        0.00;    0;          970.1;      0.0], ...         %f2x
         [0.1909;   -0.1909;  0.0;    0.;         0.0;        970.1;    -0.1909;  0.1909;   0.0;     0.;         0.0;        970.1;], ...      %t2x
         [0.0;      -0.35;    0.0;    0.;        -970.1;      0.0;       0.0;     0.35;     0.0;     0.;        -970.1;      0.0  ] ...        %t1x
         [0.0;      -0.27;    0.0;    0.;         0.0;       -970.1;     0.0;     0.27;     0.0;     0.;         0.0;        970.1;]];         %f1y 

% Xdes = [];

%If no positions are given use desired Fields. 
%U = [Bx,       By,        Bz,         dBxx,      dBxy,        dBxz,       dByy,       dByz,   mu1,    mu2]
Uc = [0.0;      0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
Ud = [[0.0;     0.0;       0.0;        0.0;        0.0;        0.1;        0.0;        0.0;    970.1;  970.1], ...
      [0.0;     0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.1;    970.1;  970.1], ...
      [0.0;     0.0;       0.0;       -0.1;        0.0;        0.0;       -0.1;        0.0;    970.1;  970.1], ...
      [0.0;    -0.01;      0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
      [0.01;    0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
      [0.0;     0.0;       0.0;        0.0;       -0.1;        0.0;        0.0;        0.0;    970.1;  970.1], ...
      [0.0;     0.0;       0.0;        0.0;        0.0;        0.0;       -0.1;        0.0;    970.1;  970.1], ...
      [0.0;     0.0;      -0.01;       0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1]];

%Constants for Field Finder
mu0 = 4*pi*1e-7;
mu = 970.1;
r_min = 0.1; %Minimum dist from magnet to origin
r_max = 1; %Maximum dist from magnet to origin
max_time = 3;

U_request = cat(2,Uc,Ud); %Creating one matrix for all the Fields we're requestion
%Checking number of diff fields/positions to go to
if (isempty(Xdes) == 1 )
    [~, numFields] = size(Ud);
else
    [~, numFields] = size(Xdes);
    numFields = numFields -1; 
end

%Matrixes with the full Path for all requested fields. 
path_points = 10;
u_planFinal =  zeros([path_points*numFields, 6]);
X_planningFinal = zeros([path_points*numFields, 12]);
X_CSVFinal = zeros([path_points*numFields, 14]);
Uc_plotFinal = zeros([10,path_points*numFields]);
Ud_plotFinal = zeros([10,path_points*numFields]);
U_pathFinal = zeros([8, path_points*numFields]);


for count = 1:numFields

    % If no desired Points are given, find end points based on desired fields
    if (isempty(Xdes) == 1 )

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

    else
        Xc = Xdes(:,count);
        Uc = field(Xc);

        Xd = Xdes(:,count + 1);
        Ud = field(Xd);
    end

     %% Creating a Linear path in polar space
    [X_planning, polarpath_1, polarpath_2] = pathCreate(Xc, Xd, path_points);


    %% Plotting Start (Current) and Desired Positions of both Magnets
    figure(1);
    subplot(round((numFields)/2) ,2, count)
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
    
    %Creating a field path 
    U_path = zeros(8, path_points);
    u_plan = zeros(6,path_points);
    
    % Creating a Vector X to write to CSV
    X_CSV = zeros(path_points,14);
    origin = [1,0,0];
    
    %Linear field interpolation
%     for i = 1:8
%        U_path(i,:) = linspace(Uc(i), Ud(i), path_points);  %Creating a field path by linearly interpolating
%     end

    %Field path with a more exponential fit to field path instead of linear
    for i =  1:8
        U_path(i,1) = Uc(i);
        U_path(i,2) = Uc(i) .* 0.4;
        U_path(i,3) = Uc(i) .* 0.1;
        U_path(i,4) = Uc(i) .* 0.05;
        U_path(i,5) = Uc(i) .* 0.02;
        U_path(i,6) = Ud(i) .* 0.6;
        U_path(i,7) = Ud(i) .* 0.9;
        U_path(i,8) = Ud(i) .* 0.95;
        U_path(i,9) = Ud(i) .* 0.975;
        U_path(i,10) = Ud(i);        
    end

    [u_plan, X_planning, X_CSV] = analyticalMewSolve(Xc, Xd, X_planning, U_path);

    u_planFinal((path_points*(count-1) + 1):(path_points*count), :) =  u_plan';
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
            ylim([-0.02 0.02]);
        else
            ylim([-0.2 0.2]);
        end

        if i == 1
            legend('Path', 'Start', 'Desired', 'Path Followed', 'FontSize', 12)
        end        
end
sgtitle("Planned Field for Full Path", 'FontSize', 24)



%% Plotting Forces and torques
m1 = [0, 0, 1];%; * 1050423;
m2 = [0, -1, 0];%; * 1050423;

% Load Cell 1
m = m1;

S = zeros([6,8]); 
S(1,1:3) = [0, -m(3), m(2)];
S(2,1:3) = [m(3), 0, -m(1)];
S(3,1:3) = [-m(2), m(1), 0];
S(4,4:8) = [m(1), m(2), m(3), 0, 0];
S(5,4:8) = [0, m(1), 0, m(2), m(3)];
S(6,4:8) = [-m(3), 0, m(1), -m(3), m(2)];

%w1_des = S*Ud_plotFinal(1:8,:);
w1_des = S*U_pathFinal;
w1_plan = S*U_final(1:8,:);

%Load Cell 2
m = m2;

S = zeros([6,8]); 
S(1,1:3) = [0, -m(3), m(2)];
S(2,1:3) = [m(3), 0, -m(1)];
S(3,1:3) = [-m(2), m(1), 0];
S(4,4:8) = [m(1), m(2), m(3), 0, 0];
S(5,4:8) = [0, m(1), 0, m(2), m(3)];
S(6,4:8) = [-m(3), 0, m(1), -m(3), m(2)];

%w2_des = S*Ud_plotFinal(1:8,:);
w2_des = S*U_pathFinal;
w2_plan = S*U_final(1:8,:);

%% Plotting Wrenches
figure();
for i = 1:6
    subplot(3, 2, i)
        plot(1:path_points*count, w1_plan(i, :)', 'k', 'LineWidth', 1.0)
        hold on
        plot(1:path_points*count, w1_des(i, :)', 'r--', 'LineWidth', 2.0)
        xlabel('Points in Path (s)', 'FontSize', 14)

        if i < 4
            ylabel(strcat('$\tau_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
        else
            ylabel(strcat('$f_', num2str(i-3),'$'), 'Interpreter', 'latex', 'FontSize', 14)
        end
       

        grid on;

        if i == 1
            legend('Wrench', 'Desired', 'FontSize', 12)
        end        

        if i <= 3
            ylim([-0.03 0.03]);
        else
            ylim([-0.15 0.15]);
        end

        %Setting plot bakground to grey for axsis we cant control
        if i == 3
            set(gca,'color',[0.6235, 0.6235, 0.6235])
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

        if i < 4
            ylabel(strcat('$\tau_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
        else
            ylabel(strcat('$f_', num2str(i-3),'$'), 'Interpreter', 'latex', 'FontSize', 14)
        end
      
        grid on;

        if i == 1
            legend('Wrench', 'Desired', 'FontSize', 12)
        end        

        if i <= 3
            ylim([-0.03 0.03]);
        else
            ylim([-0.15 0.15]);
        end

        %Setting plot bakground to grey for axsis we cant control
        if i == 2 || i == 3 || i == 6
            set(gca,'color',[0.6235, 0.6235, 0.6235])
        end
end
sgtitle("Planned Wrench w2 for Full Path", 'FontSize', 24)

%% Calculating Error between Desired and Actual

% ---- Fields ---- 
ErrField = zeros([size(U_pathFinal)]);
for i = 1:8
    ErrField(i,:) = abs((U_final(i,:) - U_pathFinal(i,:))); 
end
%norm Field Error
NormErrField(1,:) = vecnorm(ErrField(1:3,:));
%norm Gradient Error
NormErrField(2,:) = vecnorm(ErrField(4:8,:));

figure();
for i = 1:10
    subplot(5, 2, i)
    if i < 9
        plot(1:path_points*count, ErrField(i, :)', 'r', 'LineWidth', 1.0)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$ERROR U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
    elseif i == 9
        plot(1:path_points*count, NormErrField(i-8, :)', 'k', 'LineWidth', 1.0)
        title('Norm Field Error', 'FontSize', 18)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$Norm B','$'), 'Interpreter', 'latex', 'FontSize', 14)
    else
        plot(1:path_points*count, NormErrField(i-8, :)', 'k', 'LineWidth', 1.0)
        title('Norm Gradient Error', 'FontSize', 18)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$Norm dB','$'), 'Interpreter', 'latex', 'FontSize', 14)
    end
     
end
sgtitle("Error for Field", 'FontSize', 24)

%  ---- Wrenches ----
ErrWrench1 = zeros([size(w1_plan)]);
ErrWrench2 = zeros([size(w2_plan)]);
for i = 1:6
    ErrWrench1(i,:) = abs((w1_plan(i,:) - w1_des(i,:))); 
    ErrWrench2(i,:) = abs((w2_plan(i,:) - w2_des(i,:))); 
end
%norm Torque W1 Error
NormWrench1(1,:) = vecnorm(ErrWrench1(1:3,:));
%norm Force W1 Error
NormWrench1(2,:) = vecnorm(ErrWrench1(4:6,:));

%norm Torque W2 Error
NormWrench2(1,:) = vecnorm(ErrWrench2(1:3,:));
%norm Force W2 Error
NormWrench2(2,:) = vecnorm(ErrWrench2(4:6,:));

%Plotting Errors for Load Cell 1
figure();
for i = 1:8
    subplot(5, 2, i)
    if i < 7
        plot(1:path_points*count, ErrWrench1(i, :)', 'r', 'LineWidth', 1.0)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$ERROR \omega_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
    elseif i == 7
        plot(1:path_points*count, NormWrench1(i-6, :)', 'k', 'LineWidth', 1.0)
        title('Norm Torque Error', 'FontSize', 18)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$Norm \tau','$'), 'Interpreter', 'latex', 'FontSize', 14)
    else
        plot(1:path_points*count, NormWrench1(i-6, :)', 'k', 'LineWidth', 1.0)
        title('Norm Force Error', 'FontSize', 18)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$Norm f','$'), 'Interpreter', 'latex', 'FontSize', 14)
    end

    %Setting plot bakground to grey for axsis we cant control
    if i == 3
        set(gca,'color',[0.6235, 0.6235, 0.6235])
    end
     
end
sgtitle("Error for Load Cell 1", 'FontSize', 24)


%Plotting Errors for Load Cell 2
figure();
for i = 1:8
    subplot(5, 2, i)
    if i < 7
        plot(1:path_points*count, ErrWrench2(i, :)', 'r', 'LineWidth', 1.0)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$ERROR \omega_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
    elseif i == 7
        plot(1:path_points*count, NormWrench2(i-6, :)', 'k', 'LineWidth', 1.0)
        title('Norm Torque Error', 'FontSize', 18)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$Norm \tau','$'), 'Interpreter', 'latex', 'FontSize', 14)
    else
        plot(1:path_points*count, NormWrench2(i-6, :)', 'k', 'LineWidth', 1.0)
        title('Norm Force Error', 'FontSize', 18)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$Norm f','$'), 'Interpreter', 'latex', 'FontSize', 14)
    end

    %Setting plot bakground to grey for axsis we cant control
    if i == 2 || i == 3 || i == 6
        set(gca,'color',[0.6235, 0.6235, 0.6235])
    end
     
end
sgtitle("Error for Load Cell 2", 'FontSize', 24)