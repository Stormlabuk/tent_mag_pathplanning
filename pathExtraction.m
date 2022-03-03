%% Main to Plan the Path 
close all
clear all
%Adding folders and subfolders to path
addpath(genpath('actuation'));
addpath(genpath('functions'));
addpath(genpath('/home/michael/Desktop/MATLAB Scripts/MagneticPlanner/robot_tools'));

%%
%All Coordinates are in sensor frame

%U = [Bx,   By,     Bz,     dBxx,   dBxy,   dBxz,   dByy,       dByz]
Uc = [0.0;  0.0;    0.0;    0.0;    0.1;    0.0;     0.0;       0.0];    %Current Magnetic Field (Will be replaced with current position)
Ud = [0.005;  -0.003;    0.015;    0.25;    0.0;    0.0;     0.0;       0.0];

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

