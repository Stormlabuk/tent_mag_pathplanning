%% Main to Plan the Path 
close all
clear all
%Adding folders and subfolders to path
addpath(genpath('C:\Users\elmbr\OneDrive - University of Leeds\MagneticPlanner\actuation'));
addpath(genpath('C:\Users\elmbr\OneDrive - University of Leeds\MagneticPlanner\functions'));
addpath(genpath('C:\Users\elmbr\OneDrive - University of Leeds\MagneticPlanner\robot_tools'));

%%
%All Coordinates are in sensor frame

%U = [Bx,   By,     Bz,     dBxx,   dBxy,   dBxz,   dByy,       dByz]
Uc = [0.0,  0.0,    0.0,    0.0,    0.1,    0.0,     0.0,       0.0];    %Current Magnetic Field (Will be replaced with current position)
Ud = [0.0,  0.0,    0.0,    0.0,    0.0,    0.0,     0.1,       0.0];    %Desired Magnetic Field;

Xc = magPosition_alphas(Uc(4:8));    %Current Position
Xd = magPosition_alphas(Ud(4:8));    %Desired Positon

%% Plotting Current and Desired Positions of both Magnets
figure;
plot3(Xc(1),Xc(2),Xc(3), 'b*', 'MarkerSize',12);
hold on;
plot3(Xc(7),Xc(8),Xc(9), 'bo', 'MarkerSize',12);
hold on;
plot3(Xd(1),Xd(2),Xd(3), 'r*', 'MarkerSize',12);
hold on;
plot3(Xd(7),Xd(8),Xd(9), 'ro', 'MarkerSize',12);
hold on;
plot3(0,0,0,'.k','MarkerSize',40);

xlabel('x', 'FontSize',24);
ylabel('y', 'FontSize',24);
zlabel('z', 'FontSize',24);

grid on 
%% Chaning to Spherical Coordinates 

p_c1 = norm([Xc(1),Xc(2),Xc(3)]);
p_c2 = norm([Xc(7),Xc(8),Xc(9)]);
p_d1 = norm([Xd(1),Xd(2),Xd(3)]);
p_d2 = norm([Xd(7),Xd(8),Xd(9)]);

theta_c1 = atan2d(Xc(1),Xc(2));
theta_c2 = atan2d(Xc(7),Xc(8));
theta_d1 = atan2d(Xd(1),Xd(2));
theta_d2 = atan2d(Xd(7),Xd(8));

%if angle is negative change it to be positve
if theta_c1 < 0
    theta_c1 = theta_c1 + 360;
end
if theta_c2 < 0
    theta_c2 = theta_c2 + 360;
end
if theta_d1 < 0
    theta_d1 = theta_d1 + 360;
end
if theta_d2 < 0
    theta_d2 = theta_d2 + 360;
end

alpha_c1 = asind(Xc(3)/p_c1);
alpha_c2 = asind(Xc(9)/p_c2);

alpha_d1 = asind(Xd(3)/p_d1);
alpha_d2 = asind(Xd(9)/p_d2);

% %if angle is negative change it to be positve
% if alpha_c1 < 0
%     alpha_c1 = alpha_c1 + 360;
% end
% if alpha_c2 < 0
%     alpha_c2 = alpha_c2 + 360;
% end
% if alpha_d1 < 0
%     alpha_d1 = alpha_d1 + 360;
% end
% if alpha_d2 < 0
%     alpha_d2 = alpha_d2 + 360;
% end

%% Creating a Path

%Number of points in the path to be fixed for now
path_points = 30;

p_path1 = linspace(p_c1, p_d1, path_points);
p_path2 = linspace(p_c2, p_d2, path_points);

theta_path1 = linspace(theta_c1, theta_d1, path_points);
theta_path2 = linspace(theta_c2, theta_d2, path_points);

alpha_path1 = linspace(alpha_c1, alpha_d1, path_points);
alpha_path2 = linspace(alpha_c2, alpha_d2, path_points);

%Changing Back to Cartesean Space to Plot 
path_1 = zeros([path_points,3]);
path_2 = zeros([path_points,3]);

%Normalizing 
path_1(1,:) = [Xc(1), Xc(2), Xc(3)]./norm([Xc(1), Xc(2), Xc(3)]);
path_2(1,:) = [Xc(7), Xc(8), Xc(9)]./norm([Xc(7), Xc(8), Xc(9)]);

for a = 2:path_points
    path_1(a,:) = path_1(a-1,:) * rotz((theta_path1(a) - theta_path1(a-1)));  
    path_1(a,:) = path_1(a,:) * rotx((alpha_path1(a) - alpha_path1(a-1))); %"-" here as going clockwise
    
    path_2(a,:) = path_2(a-1,:) * rotz((theta_path2(a) - theta_path2(a-1)));  
    path_2(a,:) = path_2(a,:) * rotx(-(alpha_path2(a) - alpha_path2(a-1))); 
end

%Scaling by rho
%TODO: Write this with matrix multiplication instead
for a = 1:path_points
    path_1(a,:) = path_1(a,:) * p_path1(a);
    path_2(a,:) = path_2(a,:) * p_path2(a);
end

%Making last point in path the Desired Position
path_1(end,:) = [Xd(1), Xd(2), Xd(3)];
path_2(end,:) = [Xd(7), Xd(8), Xd(9)];

%% Plotting Path
hold on
plot3(path_1(:,1), path_1(:,2), path_1(:,3), ':k', 'LineWidth',2)
hold on
plot3(path_2(:,1), path_2(:,2), path_2(:,3), ':g', 'LineWidth',2)
lgd = legend('P1_{Current}','P2_{Current}','P1_{Desired}','P2_{Desired}', 'WorkSpace Center', 'Path 1', 'Path 2');
lgd.FontSize = 14;

%plotting the 0-axis
plot3(linspace(-1,1,100), zeros([1,100]), zeros([1,100]), 'k:');
plot3(zeros([1,100]), linspace(-1,1,100), zeros([1,100]), 'k:');
plot3(zeros([1,100]), zeros([1,100]), linspace(-1,1,100), 'k:');
