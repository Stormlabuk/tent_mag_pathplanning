%% Creating 2 wider circular paths around middle one
close all
clear all
addpath(genpath('C:\Users\elmbr\OneDrive - University of Leeds\Publications\MagneticPlanner\robot_tools'))

%% Showing robots on figure
robot1 = importrobot('urdf/kuka_iiwa_1.urdf','DataFormat','row');
robot2 = importrobot('urdf/kuka_iiwa_2.urdf','DataFormat','row');

q1 = [ pi/18 - pi/4, pi/6.0, 0.0, -pi/3.0 - pi/20, 0.0, pi/3.0, 0.0];
q2 = [ pi/18 - pi/4, pi/6.0, 0.0, -pi/3.0 - pi/20, 0.0, pi/3.0, 0.0];

fig = figure('Units','centimeters','Position',[10 3 21 14.85]);
s1 = subplot(1,2,1);
s1.Units = 'centimeters';
s1.FontName = 'Times New Roman';
s1.FontSize = 11;


show(robot1,q1,'Frames','off');
hold on;
show(robot2,q2,"Frames","off");



%% Creating cirlce at centre of workspace

% C = [0,0,0] ;   % center of circle 
% R = 0.1 ;    % Radius of circle 
% teta=0:0.01:2*pi ;
% x=C(1)+R*cos(teta);
% y=C(2)+R*sin(teta) ;
% z = C(3)+zeros(size(x)) ;
% patch(x,y,z,'k')
% hold on
% plot3(x,y,zeros(1,numel(x)))
% 
% [x,y,z] = sphere(2000);
% x = x*0.15;
% y = y*0.15;
% z = z*0.15;
% 
% 
% hold on;
% s1 = surf(x,y,z, 'FaceColor','#3D2824', 'FaceAlpha',0.35, 'edgecolor','none')
% hold on 
% view(89,90)
%% Starting planning

Xc = [-0.35;     -1.02;    0.19;   827.4953;   145.5150;  -485.0500;  0.35;    1.02;     0.19;   -827.4953;  -145.5150;  -485.0500];
Xd = [0.1909;   -0.1909;  0.00;   685.9643;   685.9643;   0.0;      -0.1909;  0.1909;   0.00;   -685.9643;  -685.9643;   0.0];         %f2y

path_points = 10;   %Number of points to include in path
rhoDiff = 0.075;
% Creating a Linear path in polar space
%[X_planning, polarpath_1, polarpath_2] = pathCreate(Xc, Xd, path_points);
[X_planning, polarpath_1, polarpath_2] = pathCreateVarriedRho(Xc, Xd, path_points, rhoDiff);



%% Plotting Start (Current) and Desired Positions of both Magnets
hold on
% view(90,90)
view(90,3)
ha = plot3(Xc(1),Xc(2),Xc(3), 'diamond', 'MarkerSize',10, 'MarkerFaceColor', '#0072BD', "MarkerEdgeColor",'#0072BD');
hold on;
hb = plot3(Xc(7),Xc(8),Xc(9), '.', 'MarkerSize',45, 'MarkerFaceColor', '#0072BD', "MarkerEdgeColor",'#0072BD');
hold on;
hc = plot3(Xd(1),Xd(2),Xd(3), 'diamond', 'MarkerSize',10, 'MarkerFaceColor', '#A2142F', "MarkerEdgeColor",'#A2142F');
hold on;
hd = plot3(Xd(7),Xd(8),Xd(9), '.', 'MarkerSize', 45, 'MarkerFaceColor', '#A2142F', "MarkerEdgeColor",'#A2142F');
hold on;
% he = plot3(0,0,0,'.k','MarkerSize',100);
xlim([-1 1])
ylim([-1.2 1.2])
zlim([-0.4 0.8])

xlabel('x (m)', 'FontSize', 12, 'interpreter','latex');

label_h = ylabel('y (m)', 'FontSize', 12, 'interpreter','latex');

%label_h.Position(1) = -0.13; % change horizontal position of ylabel
label_h.Position(1) = 7; % change horizontal position of ylabel
label_h.Position(2) = -0.13; % change vertical position of ylabel
label_h.Position(3) = 0; % change vertical position of ylabel


zlabel('z (m)', 'FontSize', 12,'interpreter','latex');
%title(strcat('$Path $'), 'Interpreter', 'latex', 'FontSize', 16)    

% Ax = gca;
% Ax.ZAxis.Visible = 'off';
% Ax.ZGrid = 'off';
% Ax.Color = 'none';


Ax = gca;
Ax.XAxis.Visible = 'off';
Ax.ZGrid = 'off';
Ax.Color = 'none';

%% Plotting Path
for i = 1:3
    hold on
    hf = plot3(polarpath_1(i,:,1), polarpath_1(i,:,2), polarpath_1(i,:,3), ':','Color', '#F6D46D', 'LineWidth',3);
    hold on
    hg = plot3(polarpath_2(i,:,1), polarpath_2(i,:,2), polarpath_2(i,:,3), ':','Color', '#6AD0DC', 'LineWidth',3);
end

%plotting the 0-axis
hh = plot3(linspace(-1,1,100), zeros([1,100]), zeros([1,100]), 'k:');
hi = plot3(zeros([1,100]), linspace(-1,1,100), zeros([1,100]), 'k:');
hj = plot3(zeros([1,100]), zeros([1,100]), linspace(-1,1,100), 'k:');

grid on;

% xticks([-1 -0.5 0 0.5 1])
% xticklabels({'-100','-50','0','50', '100'})

% Ax = gca;
% Ax.XAxis.Visible = 'off';
% Ax.XGrid = 'off';
% Ax.Color = 'none';


% yticks([-1 -0.5 0 0.5 1])
% yticklabels({'-1000','-500','0','500', '1000'})
% 
% xticks([-1 -0.5 0 0.5 1])
% xticklabels({'-1000','-500','0','500', '1000'})
% 
zticks([-1 -0.5 0 0.5 1])
zticklabels({'-1','-5','0','5', '1'})


% lgd = legend([ha hb hc hd hf hg he], {'$\mathbf{r_{s1}}$','$\mathbf{r_{s2}}$','$\mathbf{r_{e1}}$','$\mathbf{r_{e2}}$', '$\mathbf{\Gamma_{r1}}$', '$\mathbf{\Gamma_{r2}}$', 'Obstacle'}, 'NumColumns',4, 'Interpreter','latex');
% lgd.FontSize = 11;
% lgd.FontName = 'Times New Roman';

f = gcf;
exportgraphics(f,'PathsVarried_Side.png','Resolution',300, 'BackgroundColor','none')

%sgtitle("Planned Paths", 'FontSize', 32)

%% Showing the robots on the Figure


