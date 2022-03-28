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

%U = [Bx,   By,     Bz,     dBxx,   dBxy,   dBxz,   dByy,       dByz]
Uc = [0.0;  0.01;    0.0;    0.0;    0.0;    0.0;     0.0;       0.0];    %Current Magnetic Field (Will be replaced with current position)
Ud = [0.01;  0.0;    0.01;    0.0;    0.0;    0.0;     0.0;       0.0];

%Constants for Field Finder
mu0 = 4*pi*1e-7;
mu = 970.1;
r_min = 0.1; %Minimum dist from magnet to origin
r_max = 1; %Maximum dist from magnet to origin
max_time = 3;

[Xc,~,err_c,ic]  = field_optimise.find(Uc,mu,r_min,r_max,100,1e-12);
[Xd,~,err_d,id]  = field_optimise.find(Ud,mu,r_min,r_max,100,1e-12);
% robot_ik.displayState(Xd);


% Xc = magPosition_alphas(Uc(4:8)');    %Current Position
% Xd = magPosition_alphas(Ud(4:8)');    %Desired Positon

%% Plotting Current and Desired Positions of both Magnets
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
ylim([-1 1])
zlim([-1 1])

xlabel('x', 'FontSize',24);
ylabel('y', 'FontSize',24);
zlabel('z', 'FontSize',24);

grid on 
%% Chaning to Spherical Coordinates 

[theta_c1, p_c1, z_c1] = cart2pol(Xc(1),Xc(2),Xc(3));
[theta_c2, p_c2, z_c2] = cart2pol(Xc(7),Xc(8),Xc(9));

[theta_d1, p_d1, z_d1] = cart2pol(Xd(1),Xd(2),Xd(3));
[theta_d2, p_d2, z_d2] = cart2pol(Xd(7),Xd(8),Xd(9));
 
%If angle is negative add 360 (so that we go round the other way - taking shortest path)
if sign(theta_c1) == -1 && sign(theta_d1) == 1
    theta_c1 = theta_c1 + 2*pi;
elseif sign(theta_c1) == 1 && sign(theta_d1) == -1
    theta_d1 = theta_d1 + 2*pi;
end

if sign(theta_c2) == -1 && sign(theta_d2) == 1
    theta_c2 = theta_c2 + 2*pi;
elseif sign(theta_c2) == 1 && sign(theta_d2) == -1
    theta_d2 = theta_d2 + 2*pi;
end
% if theta_c1 < 0 
%     theta_c1 = theta_c1 + 2*pi;
% end
% if theta_c2 < 0 
%     theta_c2 = theta_c2 + 2*pi;
% end
% if theta_d1 < 0 
%     theta_d1 = theta_d1 + 2*pi;
% end
% if theta_d2 < 0 
%     theta_d2 = theta_d2 + 2*pi;
% end


%% Changing to Polar Coords
path_points = 30;

%rho is the distance from the origin to a point in the x-y plane.
p_path1 = linspace(p_c1, p_d1, path_points);
p_path2 = linspace(p_c2, p_d2, path_points);

%theta is the counterclockwise angle in the x-y plane measured in radians from the positive x-axis.
theta_path1 = linspace(theta_c1, theta_d1, path_points);
theta_path2 = linspace(theta_c2, theta_d2, path_points);

%z is the height above the x-y plane
z_polar_path1 = linspace(z_c1,z_d1,path_points);
z_polar_path2 = linspace(z_c2,z_d2,path_points);

%% Creating a path (polar)

[x_polar1,y_polar1,z_polar1] = pol2cart(theta_path1,p_path1,z_polar_path1);
[x_polar2,y_polar2,z_polar2] = pol2cart(theta_path2,p_path2,z_polar_path2);

%% Creating Vector X 

X_planning = zeros(path_points, 12);
%Setting the position of magnet 1
X_planning(:,1:3) = [x_polar1; y_polar1; z_polar1]';
%Setting the position of magnet 2
X_planning(:,7:9) = [x_polar2; y_polar2; z_polar2]';

%Setting the orientation of magnet 1 (without optimization for now)
X_planning(1:path_points/2, 4:6) = repmat(Xc(4:6),1, path_points/2)';
X_planning(path_points/2 + 1: end, 4:6) = repmat(Xd(4:6),1, path_points/2)';
%Setting the orientation of magnet 2 (without optimization for now)
X_planning(1:path_points/2, 10:12) = repmat(Xc(10:12),1, path_points/2)';
X_planning(path_points/2 + 1: end, 10:12) = repmat(Xd(10:12),1, path_points/2)';

%% Getting field at everypoint in path 
U_path = zeros(10, path_points);

for a = 1:path_points
    U_path(:,a) = field(X_planning(a,:));
end

%% Plotting Path
hold on
hf = plot3(x_polar1, y_polar1, z_polar1, ':k', 'LineWidth',2);
hold on
hg = plot3(x_polar2, y_polar2, z_polar2, ':g', 'LineWidth',2);

%plotting the 0-axis
hh = plot3(linspace(-1,1,100), zeros([1,100]), zeros([1,100]), 'k:');
hi = plot3(zeros([1,100]), linspace(-1,1,100), zeros([1,100]), 'k:');
hj = plot3(zeros([1,100]), zeros([1,100]), linspace(-1,1,100), 'k:');


lgd = legend([ha hb hc hd he hf hg], 'P1_{Current}','P2_{Current}','P1_{Desired}','P2_{Desired}', 'WorkSpace Center', 'Path 1', 'Path 2');
lgd.FontSize = 14;

%% Plotting Field at each point in path
Uc_plot = repmat(Uc, 1, 30);
Ud_plot = repmat(Ud, 1, 30);

figure();

for i = 1:3
    subplot(4, 2, i)
        plot(1:30, U_path(i, :), '--', 'LineWidth', 3.0)
        hold on
        plot(1:30, Uc_plot(i, :), 'go', 'LineWidth', 1.0)
        hold on
        plot(1:30, Ud_plot(i, :), 'ro', 'LineWidth', 1.0)
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
% %Experimenting with sin waves
% x_sin = 0:0.2125:2*pi;
% U_path(1,:) = 0.02*sin(x_sin);% + 0.02;

%finding mew analytically
mu0 = 4*pi*1e-7; % air magnetic permeability
for i = 1:path_points
    r1 = X_planning(i,1:3)';
    ro1 = norm(r1);
    r2 = X_planning(i,7:9)';
    ro2 = norm(r2);

    M1 = (mu0/(4*pi*norm(ro1)^3)) * (3*(r1/ro1)*(r1/ro1)' - eye(3));
    M2 = (mu0/(4*pi*norm(ro2)^3)) * (3*(r2/ro2)*(r2/ro2)' - eye(3));

    M = [M1, M2];
    %finding the magnetic moment
    u_plan(:,i) = pinv(M) * U_path(1:3,i);
    %normalizing
    u_plan(1:3,i) = u_plan(1:3,i)/norm(u_plan(1:3,i)) * 970.1;
    u_plan(4:6,i) = u_plan(4:6,i)/norm(u_plan(4:6,i)) * 970.1;

    %Placing in Final X Matrix
    X_planning(i, 4:6) = u_plan(1:3,i);
    X_planning(i, 10:12) = u_plan(4:6,i);

    %Vector to write to CSV
    X_CSV(i,1:3) = X_planning(i,1:3);
    X_CSV(i,8:10) = X_planning(i,7:9);
    %changing from moment to rot mat
    C = cross(origin, u_plan(1:3,i)) ; 
    D = dot(origin, u_plan(1:3,i)) ;
    NP0 = norm(origin) ; % used for scaling
    if ~all(C==0) % check for colinearity    
        Z = [0 -C(3) C(2); C(3) 0 -C(1); -C(2) C(1) 0] ; 
        R = (eye(3) + Z + Z^2 * (1-D)/(norm(C)^2)) / NP0^2 ; % rotation matrix
    else
        R = sign(D) * (norm(u_plan(1:3,i)) / NP0) ; % orientation and scaling
    end
    % R is the rotation matrix from p0 to p1, so that (except for round-off errors) ...
    X_CSV(i, 4:7) = rotm2quat(R);

    %changing from moment to rot mat
    C = cross(origin, u_plan(4:6,i)) ; 
    D = dot(origin, u_plan(4:6,i)) ;
    NP0 = norm(origin) ; % used for scaling
    if ~all(C==0) % check for colinearity    
        Z = [0 -C(3) C(2); C(3) 0 -C(1); -C(2) C(1) 0] ; 
        R = (eye(3) + Z + Z^2 * (1-D)/(norm(C)^2)) / NP0^2 ; % rotation matrix
    else
        R = sign(D) * (norm(u_plan(4:6,i)) / NP0) ; % orientation and scaling
    end
    % R is the rotation matrix from p0 to p1, so that (except for round-off errors) ...
    X_CSV(i, 11:14) = rotm2quat(R);



end


%% Optimisation
% 
% lb = zeros([6,1]);
% ub = 970.1 * ones([6,1]);
% m0 = [Xc(4:6); Xc(10:12)];
% [c, ceq] = nlcon(m0);
% 
% A = [];
% B = [];
% Aeq = [];
% Beq = [];
% nonlincon = @nlcon;
% 
% fval(1) = 0;
% 
% m = zeros([6,path_points]);
% m(:,1) = m0;
% for i = 2:path_points
%     rOld = [X_planning(i-1,1:3)' ; X_planning(i-1,7:9)']; 
%     r = [X_planning(i,1:3)' ; X_planning(i,7:9)'];
%     mOld = [X_planning(i-1,4:6)' ; X_planning(i-1,10:12)'];
%     m0 = m(:,i-1);
% 
%    [m(:,i),fval(i)] = fmincon(@(m)fieldObjective(m, mOld, r, rOld, Ud), m0, A, B, Aeq, Beq, lb, ub, nonlincon);
% end
% 
% %Setting the last value to that of the optimisation 
% m(:,end) = [Xd(4:6);Xd(10:12)];
% 
% % Placing the orientation inside the X vector;
% X_planning(:,4:6) = m(1:3,:)';
% X_planning(:,10:12) = m(4:6,:)';
% 
% % Xnew = [r(1:3); m(1:3,30); r(4:6); m(4:6,30)]; 
% % fieldnew = field(Xnew);

%% Plotting Field at each point in path
U_final = zeros(10, path_points);

for a = 1:path_points
    U_final(:,a) = field(X_planning(a,:));
end

figure();
for i = 1:3
    subplot(4, 2, i)
        plot(1:30, U_path(i, :), '--', 'LineWidth', 1.0)
        hold on
        plot(1:30, Uc_plot(i, :), 'go', 'LineWidth', 1.0)
        hold on
        plot(1:30, Ud_plot(i, :), 'ro', 'LineWidth', 1.0)
        hold on
        plot(1:30, U_final(i,:), 'k--', 'LineWidth', 3.0)
        grid on
        xlabel('Points in Path (s)')
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex')  

        if i == 1
            legend('Path', 'Start', 'Desired', 'Path Followed')
        end        
end
sgtitle("After Optimisation", 'FontSize', 24)


% figure;
% plot(fval);
% title("Error", 'FontSize',20)
