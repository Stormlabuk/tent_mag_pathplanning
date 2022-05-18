function [X_planning, polarpath_1, polarpath_2] = pathCreateVarriedRho(Xc, Xd, path_points, rhoDiff)
%TODO: Scale for more (or variable) number of paths not just 3

%Creating a paths following a curved trajectory each a radius rho appart
%First and last points still lie on original Curve
%polarpath_# is an array with paths of varied rho for that robot

% Chaning to Spherical Coordinates 
[theta_c1, p_c1, z_c1] = cart2pol(Xc(1),Xc(2),Xc(3));
[theta_c2, p_c2, z_c2] = cart2pol(Xc(7),Xc(8),Xc(9));

[theta_d1, p_d1, z_d1] = cart2pol(Xd(1),Xd(2),Xd(3));
[theta_d2, p_d2, z_d2] = cart2pol(Xd(7),Xd(8),Xd(9));
 
%If angle is negative add 360 (so that we go round the other way - taking shortest path)
%need to round answers, otherwise when theta is close to 0 (<e-5) will give
%errors
if sign(round(theta_c1,4)) == -1 && sign(round(theta_d1,4)) == 1
    theta_c1 = theta_c1 + 2*pi;
elseif sign(round(theta_c1,4)) == 1 && sign(round(theta_d1,4)) == -1
    theta_d1 = theta_d1 + 2*pi;
end

if sign(round(theta_c2,4)) == -1 && sign(round(theta_d2,4)) == 1
    theta_c2 = theta_c2 + 2*pi;
elseif sign(round(theta_c2,4)) == 1 && sign(round(theta_d2,4)) == -1
    theta_d2 = theta_d2 + 2*pi;
end


%Preparing initial value of rho
rho_c1 = p_c1 - rhoDiff;
rho_c2 = p_c2 - rhoDiff;
rho_d1 = p_d1 - rhoDiff;
rho_d2 = p_d2 - rhoDiff;

% Creating Vector X 
X_planning = zeros(3, path_points, 12);

% Changing to Polar Coords
for i = 1:3
    %rho is the distance from the origin to a point in the x-y plane.
    p_path1 = linspace(rho_c1, rho_d1, path_points);
    p_path2 = linspace(rho_c2, rho_d2, path_points);

    rho_c1 = rho_c1 + rhoDiff;
    rho_c2 = rho_c2 + rhoDiff;
    rho_d1 = rho_d1 + rhoDiff;
    rho_d2 = rho_d2 + rhoDiff;
    
    %theta is the counterclockwise angle in the x-y plane measured in radians from the positive x-axis.
    theta_path1 = linspace(theta_c1, theta_d1, path_points);
    theta_path2 = linspace(theta_c2, theta_d2, path_points);
    
    %z is the height above the x-y plane
    z_polar_path1 = linspace(z_c1,z_d1,path_points);
    z_polar_path2 = linspace(z_c2,z_d2,path_points);
    
    % Creating a path (polar)
    [x_polar1 ,y_polar1 ,z_polar1] = pol2cart(theta_path1,p_path1,z_polar_path1);
    [x_polar2 ,y_polar2 ,z_polar2] = pol2cart(theta_path2,p_path2,z_polar_path2);
    
    polarpath_1(i,:,:) = [x_polar1 ; y_polar1 ; z_polar1]';
    polarpath_2(i,:,:) = [x_polar2 ; y_polar2 ; z_polar2]';
    
    % Creating Vector X 
    %Setting the position of magnet 1
    X_planning(i,:,1:3) = [x_polar1; y_polar1; z_polar1]';
    %Setting the position of magnet 2
    X_planning(i,:,7:9) = [x_polar2; y_polar2; z_polar2]';
    
    %Setting the orientation of magnet 1 (without optimization for now)
    X_planning(i, 1:path_points/2, 4:6) = repmat(Xc(4:6),1, path_points/2)';
    X_planning(i, path_points/2 + 1: end, 4:6) = repmat(Xd(4:6),1, path_points/2)';
    %Setting the orientation of magnet 2 (without optimization for now)
    X_planning(i, 1:path_points/2, 10:12) = repmat(Xc(10:12),1, path_points/2)';
    X_planning(i, path_points/2 + 1: end, 10:12) = repmat(Xd(10:12),1, path_points/2)';

end

%making sure last and first points of all routes are the desired
%Start Pos
polarpath_1(1,1,1:3) = polarpath_1(2,1,1:3);
polarpath_1(3,1,1:3) = polarpath_1(2,1,1:3);
polarpath_2(1,1,1:3) = polarpath_2(2,1,1:3);
polarpath_2(3,1,1:3) = polarpath_2(2,1,1:3);


%End Pos
polarpath_1(1,end,1:3) = polarpath_1(2,end,1:3);
polarpath_1(3,end,1:3) = polarpath_1(2,end,1:3);
polarpath_2(1,end,1:3) = polarpath_2(2,end,1:3);
polarpath_2(3,end,1:3) = polarpath_2(2,end,1:3);

%Same for X Vector
%Start
X_planning(1,1,:) = X_planning(2,1,:);
X_planning(3,1,:) = X_planning(2,1,:);
%End
X_planning(1,end,:) = X_planning(2,end,:);
X_planning(3,end,:) = X_planning(2,end,:);

%Returning 9 Different Versions of X by paring different paths for each
%robot (example -rho for robot 1 and +rho for robot 2).
X_planningReturn = zeros(12, path_points, 12);
%TODO: Automate this
%robot 1
X_planningReturn(1,:,1:6) = X_planning(1,:,1:6);
X_planningReturn(2,:,1:6) = X_planning(1,:,1:6);
X_planningReturn(3,:,1:6) = X_planning(1,:,1:6);
X_planningReturn(4,:,1:6) = X_planning(2,:,1:6);
X_planningReturn(5,:,1:6) = X_planning(2,:,1:6);
X_planningReturn(6,:,1:6) = X_planning(2,:,1:6);
X_planningReturn(7,:,1:6) = X_planning(3,:,1:6);
X_planningReturn(8,:,1:6) = X_planning(3,:,1:6);
X_planningReturn(9,:,1:6) = X_planning(3,:,1:6);

%robot2
X_planningReturn(1,:,7:12) = X_planning(1,:,7:12);
X_planningReturn(2,:,7:12) = X_planning(2,:,7:12);
X_planningReturn(3,:,7:12) = X_planning(3,:,7:12);
X_planningReturn(4,:,7:12) = X_planning(1,:,7:12);
X_planningReturn(5,:,7:12) = X_planning(2,:,7:12);
X_planningReturn(6,:,7:12) = X_planning(3,:,7:12);
X_planningReturn(7,:,7:12) = X_planning(1,:,7:12);
X_planningReturn(8,:,7:12) = X_planning(2,:,7:12);
X_planningReturn(9,:,7:12) = X_planning(3,:,7:12);

% %Doing Same for Polar Path Points
% polarpathReturn_1 = zeros(9,path_points,3);
% polarpathReturn_2 = zeros(9,path_points,3);
% 
% polarpathReturn_1(1,:,:) = polarpath_1(1,:,:);
% polarpathReturn_1(2,:,:) = polarpath_1(1,:,:);
% polarpathReturn_1(3,:,:) = polarpath_1(1,:,:);
% polarpathReturn_1(4,:,:) = polarpath_1(2,:,:);
% polarpathReturn_1(5,:,:) = polarpath_1(2,:,:);
% polarpathReturn_1(6,:,:) = polarpath_1(2,:,:);
% polarpathReturn_1(7,:,:) = polarpath_1(3,:,:);
% polarpathReturn_1(8,:,:) = polarpath_1(3,:,:);
% polarpathReturn_1(9,:,:) = polarpath_1(3,:,:);
% 
% polarpathReturn_2(1,:,:) = polarpath_2(1,:,:);
% polarpathReturn_2(2,:,:) = polarpath_2(2,:,:);
% polarpathReturn_2(3,:,:) = polarpath_2(3,:,:);
% polarpathReturn_2(4,:,:) = polarpath_2(1,:,:);
% polarpathReturn_2(5,:,:) = polarpath_2(2,:,:);
% polarpathReturn_2(6,:,:) = polarpath_2(3,:,:);
% polarpathReturn_2(7,:,:) = polarpath_2(1,:,:);
% polarpathReturn_2(8,:,:) = polarpath_2(2,:,:);
% polarpathReturn_2(9,:,:) = polarpath_2(3,:,:);


end