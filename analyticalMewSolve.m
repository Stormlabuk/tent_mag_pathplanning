function [u_plan, X_planning, X_CSV] = analyticalMewSolve(Xc, Xd, X_planning, U_path)
%Function to solve for mew analytically for a given field and path

[path_points, ~] = size(X_planning);   %number of points in path
u_plan = zeros(6,path_points);

% Creating a Vector X to write to CSV
X_CSV = zeros(path_points,14);
origin = [1,0,0];

rot45 = rotz(-45);

%finding mew analytically
mu0 = 4*pi*1e-7; % air magnetic permeability
for i = 1:path_points
    r1 = X_planning(i,1:3)';
    ro1 = norm(r1);
    r2 = X_planning(i,7:9)';
    ro2 = norm(r2);

    %Field 
    M1 = (mu0/(4*pi*norm(ro1)^3)) * (3*(r1/ro1)*(r1/ro1)' - eye(3));
    M2 = (mu0/(4*pi*norm(ro2)^3)) * (3*(r2/ro2)*(r2/ro2)' - eye(3));

    M = [M1, M2];

    %Gradients 
    N = [];
        for j = 1:6

            %Required magnet orientation
            mu_sub = zeros(6,1);
            mu_sub(j) = 1;

            Id = eye(3);

            %Dipole Model for Gradients
            jacob = (3* mu0/(4*pi*norm(ro1)^4)) * ...
                ((Id - 5*((r1/ro1)*(r1/ro1)'))*((r1/ro1).'*mu_sub(1:3)) + mu_sub(1:3)*(r1/ro1)' + (r1/ro1)*mu_sub(1:3)') ...
                + (3* mu0/(4*pi*norm(ro2)^4)) * ((Id - 5*((r2/ro2)*(r2/ro2)'))*((r2/ro2).'*mu_sub(4:6)) + mu_sub(4:6)*(r2/ro2)' + (r2/ro2)*mu_sub(4:6)');
            
            dB = [jacob(1, 1); jacob(1, 2); jacob(1, 3); jacob(2, 2); jacob(2, 3)];
            N = [N, dB];
        end


    %finding the magnetic moment
    P = [M; N];

    u_plan(:,i) = pinv(P) * U_path(:,i);
    %normalizing
    %Setting first point to one found by optisation
    if i == 1
        u_plan(1:3,i) = Xc(4:6);
        u_plan(4:6,i) = Xc(10:12);
    elseif i == path_points %Setting last point to one found by optisation
        u_plan(1:3,i) = Xd(4:6);
        u_plan(4:6,i) = Xd(10:12);
    else
        u_plan(1:3,i) = u_plan(1:3,i)/norm(u_plan(1:3,i)) * 970.1;
        u_plan(4:6,i) = u_plan(4:6,i)/norm(u_plan(4:6,i)) * 970.1;
    end

%     u_plan(:,i) = pinv(M) * U_path(1:3,i);
%     %normalizing
%     u_plan(1:3,i) = u_plan(1:3,i)/norm(u_plan(1:3,i)) * 970.1;
%     u_plan(4:6,i) = u_plan(4:6,i)/norm(u_plan(4:6,i)) * 970.1;

    %Placing in Final X Matrix
    X_planning(i, 4:6) = u_plan(1:3,i);
    X_planning(i, 10:12) = u_plan(4:6,i);

    %Vector to write to CSV -- Rotating to New Reference Frame
    X_CSV(i,1:3) = rot45*X_planning(i,1:3)';
    X_CSV(i,8:10) = rot45*X_planning(i,7:9)';
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
    %Rotating to new reference frame
    Rnew = rot45 * R;
    X_CSV(i, 4:7) = rotm2quat(Rnew);

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
    %Rotating to new reference frame
    Rnew = rot45 * R;

    %%Dont know why was changing rotation of robot 1 again
%     X_CSV(i, 4:7) = rotm2quat(Rnew);
%     X_CSV(i, 11:14) = rotm2quat(R);

    %%Changed to this on the 10/05/2022
    %X_CSV(i, 4:7) = rotm2quat(Rnew);
    X_CSV(i, 11:14) = rotm2quat(Rnew);

end


end
