%% Creating 2 wider circular paths around middle one
close all
clear all


Xc = [-0.35;     -1.02;    0.19;   827.4953;   145.5150;  -485.0500;  0.35;    1.02;     0.19;   -827.4953;  -145.5150;  -485.0500];
Xd = [0.1909;   -0.1909;  0.00;   685.9643;   685.9643;   0.0;      -0.1909;  0.1909;   0.00;   -685.9643;  -685.9643;   0.0];         %f2y

path_points = 10;   %Number of points to include in path
rhoDiff = 0.075;
% Creating a Linear path in polar space
%[X_planning, polarpath_1, polarpath_2] = pathCreate(Xc, Xd, path_points);
[X_planning, polarpath_1, polarpath_2] = pathCreateVarriedRho(Xc, Xd, path_points, rhoDiff);

%% Plotting Start (Current) and Desired Positions of both Magnets
figure(1);
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
title(strcat('$Path $'), 'Interpreter', 'latex', 'FontSize', 16)           

%% Plotting Path
for i = 1:3
    hold on
    hf = plot3(polarpath_1(i,:,1), polarpath_1(i,:,2), polarpath_1(i,:,3), ':k', 'LineWidth',2);
    hold on
    hg = plot3(polarpath_2(i,:,1), polarpath_2(i,:,2), polarpath_2(i,:,3), ':g', 'LineWidth',2);
end

%plotting the 0-axis
hh = plot3(linspace(-1,1,100), zeros([1,100]), zeros([1,100]), 'k:');
hi = plot3(zeros([1,100]), linspace(-1,1,100), zeros([1,100]), 'k:');
hj = plot3(zeros([1,100]), zeros([1,100]), linspace(-1,1,100), 'k:');

grid on;
lgd = legend([ha hb hc hd he hf hg], 'P1_{Current}','P2_{Current}','P1_{Desired}','P2_{Desired}', 'WorkSpace Center', 'Path 1', 'Path 2');
lgd.FontSize = 14;
sgtitle("Paths to be Taken", 'FontSize', 24)
    