function [X_planning, polarpath_1, polarpath_2] = pathCreate(Xc, Xd, path_points)
%Creating a path following a curved trajectory

% Chaning to Spherical Coordinates 
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


% Changing to Polar Coords
%rho is the distance from the origin to a point in the x-y plane.
p_path1 = linspace(p_c1, p_d1, path_points);
p_path2 = linspace(p_c2, p_d2, path_points);

%theta is the counterclockwise angle in the x-y plane measured in radians from the positive x-axis.
theta_path1 = linspace(theta_c1, theta_d1, path_points);
theta_path2 = linspace(theta_c2, theta_d2, path_points);

%z is the height above the x-y plane
z_polar_path1 = linspace(z_c1,z_d1,path_points);
z_polar_path2 = linspace(z_c2,z_d2,path_points);

% Creating a path (polar)
[x_polar1,y_polar1,z_polar1] = pol2cart(theta_path1,p_path1,z_polar_path1);
[x_polar2,y_polar2,z_polar2] = pol2cart(theta_path2,p_path2,z_polar_path2);

polarpath_1 = [x_polar1; y_polar1; z_polar1]';
polarpath_2 = [x_polar2; y_polar2; z_polar2]';

% Creating Vector X 
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

end