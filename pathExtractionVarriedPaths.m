%% Main to Plan the Path - Gives Multiple Paths as Option and Allows Planner to Chose Best
close all
clear all
%Adding folders and subfolders to path
addpath(genpath('actuation'));
addpath(genpath('functions'));
%addpath(genpath('/home/michael/Desktop/MATLAB Scripts/MagneticPlanner/robot_tools'));
addpath(genpath('C:\Users\elmbr\OneDrive - University of Leeds\Publications\MagneticPlanner'));
%%
%All Coordinates are in sensor frame

%% Load Cell Experiments
%Enter Desired End Effector Positions
% Xdes = [[-0.35;     -1.02;    0.19;   827.4953;   145.5150;  -485.0500;  0.35;    1.02;     0.19;   -827.4953;  -145.5150;  -485.0500], ...
%          [0.1909;   -0.1909;  0.00;   685.9643;   685.9643;   0.0;      -0.1909;  0.1909;   0.00;   -685.9643;  -685.9643;   0.0], ...         %f2y
%          [0.35;      0;       0.0;    970.1;      0.0;        0.0;      -0.35;    0.0;      0.00;    970.1;      0.0;        0.0], ...         %t1y
%          [0.1909;    0;      -0.1909; -686.0814;   0.0;      -685.8472; -0.1909;  0;        0.1909;  685.8472;   0.0;        686.0613], ...    %f1z
%          [0.27;      0;       0.0;    0.;        -0.0;        970.1;    -0.27;    0;        0.00;    0;          0.0;       -970.1], ...       %f1x
%          [0.27;      0;       0.0;    0.;        -970.1;      0.0;      -0.27;    0;        0.00;    0;          970.1;      0.0], ...         %f2x
%          [0.1909;   -0.1909;  0.0;    0.;         0.0;        970.1;    -0.1909;  0.1909;   0.0;     0.;         0.0;        970.1;], ...      %t2x
%          [0.0;      -0.35;    0.0;    0.;        -970.1;      0.0;       0.0;     0.35;     0.0;     0.;        -970.1;      0.0  ] ...        %t1x
%          [0.0;      -0.27;    0.0;    0.;         0.0;       -970.1;     0.0;     0.27;     0.0;     0.;         0.0;        970.1;]];         %f1y 
% 
% Xdes = [[0.1175205;	-0.12847269;	0.074631775;  -592.4013; -677.1489; -362.8004;	-0.189308796;	0.189308796;	0.09479692; -520.7276; -760.9802;  301.4066], ...
%         [0.0603791;	-0.14393233;   -0.102485047;   639.9926;  370.1161; -628.1063;   0.024473661;	0.228785146;   -0.102485047; 639.9926;  370.1161;  628.1063]];



%Fussili Experiments
%Enter Desired End Effector Positions
% Xdes =  [[-0.969;	-0.474;	0.19;	-841.6736;	35.8330;	-481.0359;	0.969;	0.474;	0.19;	-688.0980; 482.1743; -484.8949], ...
%          [0.423391644655197	-0.0366974216988890	-1.04771207846113e-18	937.966372357637	-247.614810393592	-7.06924087274634e-15	-0.423391644655197	0.0366974216988884	1.04786011268186e-18	937.966372357639	-247.614810393585	-7.07057309989643e-15]'];

% %Planning Square Gradients Experiments
% %Enter Desired End Effector Positions
% Xdes =  [[-0.969;	-0.474;	0.19;	-841.6736;	35.8330;	-481.0359;	0.969;	0.474;	0.19;	-688.0980; 482.1743; -484.8949], ...
%          [1.95E-01	-1.95E-01	1.24E-05	25.67503028	-2.57E+01	9.69E+02 -1.95E-01	1.95E-01	1.24E-05	-25.67503028	2.57E+01	-9.69E+02]', ...
%          [2.53E-01	-2.22E-16	1.14E-05	3.85E+01	-1.31E-13	9.69E+02 -2.53E-01	-1.94E-16	1.14E-05	-3.85E+01	-8.17E-14	-9.69E+02]', ...
%          [-1.94E-16	-2.53E-01	1.14E-05	-8.17E-14	-3.85E+01	-9.69E+02	-1.94E-16	2.53E-01	1.14E-05	-1.31E-13	3.85E+01	969.3352677]', ...
%          [2.53E-01	8.33E-17	1.14E-05	38.51167092	5.68E-14	-9.69E+02 -2.53E-01	8.33E-17	1.14E-05	-38.51167092	5.68E-14	9.69E+02]', ...
%          [-0.195268873	0.195268873	1.24E-05	-25.67503028	25.67503028	-969.4202369	0.195268873	-0.195268873	1.24E-05	25.67503028	-25.67503028	969.4202369]'];

%Planning Square Field Experiments
%Enter Desired End Effector Positions
% Xdes =  [[-0.969;	-0.474;	0.19;	-841.6736;	35.8330;	-481.0359;	0.969;	0.474;	0.19;	-688.0980; 482.1743; -484.8949], ...
%          [0.183056432	-0.183056432	7.04E-19	-306.7725559	-920.3176674	3.54E-15	-0.183056419	0.183056419	-7.04E-19	-306.7725559	-920.3176674	3.54E-15]', ...
%          [0.183056432	-0.183056432	-5.24E-19	-920.3176674	-306.7725559	2.63E-15	-0.183056419	0.183056419	5.24E-19	-920.3176674	-306.7725559	2.63E-15]', ...
%          [0.183056432	-0.183056432	9.88E-20	306.7725559	920.3176674	-4.97E-16	-0.183056419	0.183056419	-9.88E-20	306.7725559	920.3176674	-4.97E-16]', ...
%          [0.183056432	-0.183056432	2.05E-19	920.3176674	306.7725559	1.03E-15	-0.183056419	0.183056419	-2.05E-19	920.3176674	306.7725559	1.03E-15]', ...
%          [0.183056432	-0.183056432	7.04E-19	-306.7725559	-920.3176674	3.54E-15	-0.183056419	0.183056419	-7.04E-19	-306.7725559	-920.3176674	3.54E-15]'];
% 

%Planning Fig 8 Field Experiments
%Enter Desired End Effector Positions
% Xdes =  [[-0.969;	-0.474;	0.19;	-841.6736;	35.8330;	-481.0359;	0.969;	0.474;	0.19;	-688.0980; 482.1743; -484.8949], ...
%          [0.23063665	-0.23063665	    3.11E-19	920.3176674	306.7725559	1.24E-15 -0.230636638	0.230636638	-3.11E-19	920.3176674	306.7725559	1.24E-15]', ...
%          [0.239375661	-0.239375661	5.17E-12	685.9642884	-685.9642884	5.68E-08	-0.239375648	0.239375648	3.46E-12	685.9642885	-685.9642884	-4.21E-08]', ...
%          [0.23063665	-0.23063665	    9.82E-19	-306.7725559	-920.3176674	3.92E-15	-0.230636638	0.230636638	-9.82E-19	-306.7725559	-920.3176674	3.92E-15]', ...
%          [0.23063665	-0.23063665 	5.53E-20	306.7725559	920.3176674	-2.21E-16	-0.230636638	0.230636638	-5.53E-20	306.7725559	920.3176674	-2.21E-16]', ...
%          [0.239375281	-0.239376041	5.37E-07	-685.961022	685.9675548	-0.004619191	-0.239375268	0.239376028	-5.37E-07	-685.9610221	685.9675547	-0.004619345]', ...
%          [0.23063665	-0.23063665     -5.16E-19	-920.3176674	-306.7725559	2.06E-15   -0.230636638	0.230636638	5.16E-19	-920.3176674	-306.7725559	2.06E-15]'];


%Planning ZigZag Experiments
%Enter Desired End Effector Positions
% Xdes =  [[-0.969;	-0.474;	0.19;	-841.6736;	35.8330;	-481.0359;	0.969;	0.474;	0.19;	-688.0980; 482.1743; -484.8949], ...
%          [-0.195268873	-0.195268873	1.24E-05	-25.67503028	-25.67503028	969.4202369	0.195268873	0.195268873	1.24E-05	25.67503028	25.67503028	-969.4202369]', ...
%          [-0.130971866	-0.261943731	1.32E-05	-18.28791159	-32.39258624	9.69E+02	0.130971866	0.261943731	1.32E-05	1.83E+01	32.39258624	-9.69E+02]', ...
%          [-0.261943731	0.130971866	1.32E-05	-32.39258624	18.28791159	9.69E+02	0.261943731	-0.130971866	1.32E-05	3.24E+01	-18.28791159	-9.69E+02]', ...
%          [-0.195268873	0.195268873	1.24E-05	-25.67503028	25.67503028	-9.69E+02	0.195268873	-0.195268873	1.24E-05	2.57E+01	-25.67503028	9.69E+02]', ...
%          [0.130971866	-0.261943731	1.32E-05	18.28791159	-32.39258624	-9.69E+02	-0.130971866	0.261943731	1.32E-05	-1.83E+01	32.39258624	9.69E+02]', ...
%          [-0.261943731	-0.130971866	1.32E-05	-32.39258624	-18.28791159	-9.69E+02	0.261943731	0.130971866	1.32E-05	3.24E+01	18.28791159	9.69E+02]',...
%          [-0.195268873	-0.195268873	1.24E-05	-25.67503028	-25.67503028	-9.69E+02	0.195268873	0.195268873	1.24E-05	2.57E+01	25.67503028	9.69E+02]'];

% %Planning Bx By Experimeants
% Xdes =  [[-0.969;	    -0.474;	        0.19;	     -841.6736;	       35.8330; 	-481.0359;	    0.969;	        0.474;	0.19;	    -688.0980; 482.1743; -484.8949], ...
%          [0.233345238	-0.233345238	0             685.9643        -685.9643         0            -0.233345238	0.233345238	    0       685.9643    -685.9643         0]', ...
%          [-0.233345238	-0.233345238	0             685.9643         685.9643         0           0.233345238	    0.233345238	    0       685.9643  685.9643         0 ]', ...
%          [0.233345238	-0.233345238	0             -685.9643        685.9643         0            -0.233345238	0.233345238	    0       -685.9643    685.9643         0]', ...
%          [-0.233345238	-0.233345238	0             -685.9643       -685.9643         0           0.233345238	    0.233345238	    0      -685.9643  -685.9643         0 ]', ...
%          [0.233345238	-0.233345238	0             685.9643        -685.9643         0            -0.233345238	0.233345238	    0       685.9643    -685.9643         0]'];

%Planning Brain Phantom Experiments
% Xdes =  [[-0.969;	    -0.474;	        0.19;	     -841.6736;	       35.8330; 	-481.0359;	    0.969;	        0.474;	0.19;	    -688.0980; 482.1743; -484.8949], ...
%          [0.159955768	-0.159955768	-2.71E-15	0.567972094	3.62E-01	9.70E+02	-2.31E-01	2.31E-01	-2.71E-15	0.567972094	0.361522937	-9.70E+02]',...
%          [0.154076442	-0.154076442	-4.37E-12	-685.8002164	-2.29E+02	6.47E+02	-1.54E-01	1.54E-01	4.37E-12	-685.8002164	-228.8598536	6.47E+02]', ...
%          [0.1258   -0.1758    0.0000  -685.9540  -685.9745  0  -0.175797206645719  0.125797206645719   -6.32E-12   -685.9540  -685.9745   0]', ...
%          [-0.969;	    -0.474;	        0.19;	     -841.6736;	       35.8330; 	-481.0359;	    0.969;	        0.474;	0.19;	    -688.0980; 482.1743; -484.8949]];
% 

% Zero Position
% [-0.969;	-0.474;	0.19;	-841.6736;	35.8330;	-481.0359;	0.969;	0.474;	0.19;	-688.0980; 482.1743; -484.8949]

%If no positions are given use desired Fields. 
% U = [Bx,       By,        Bz,         dBxx,      dBxy,        dBxz,       dByy,       dByz,   mu1,    mu2]
% Uc = [0.0;      0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
% Ud = [[0.005;   0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;    -0.005;     0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;     0.0;      -0.0075;     0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;     0.0;       0.0;        0.0;        0.15;       0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;     0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1]];
%       [0.01;    0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;     0.0;       0.0;        0.0;       -0.1;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;     0.0;       0.0;        0.0;        0.0;        0.0;       -0.1;        0.0;    970.1;  970.1], ...
%       [0.0;     0.0;      -0.01;       0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1]];
 
% Uc = [0.0;      0.0;      0.0;         0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
% Ud = [[0.0;     0.0;      0.0;         0.0;        0.0;        0.1;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;     0.0;      0.0;         0.0;        0.0;        0.1;        0.0;        0.1;    970.1;  970.1], ...
%       [0.0;     0.0;      0.0;         0.0;        0.0;       -0.1;        0.0;        0.1;    970.1;  970.1], ...
%       [0.0;     0.0;      0.0;         0.0;        0.0;       -0.1;        0.0;       -0.1;    970.1;  970.1], ...
%       [0.0;     0.0;      0.0;         0.0;        0.0;        0.1;        0.0;        0.0;    970.1;  970.1]];
% 

% % %Planning Square Experiments
% Uc = [ 0.0;    0.0;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
% Ud = [[0.01;   0.0;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.01;   0.01;   0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%      [-0.01;   0.01;   0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%      [-0.01;  -0.01;   0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.01;  -0.01;   0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.01;   0.0;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1]];
Uc = [ 0.0;    0.0;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
Ud = [0.0;   0.0;    0.0;        0.1;        0.0;        0.0;        0.1;        0.0;    970.1;  970.1];
Xdes = [];


% %Planning Figure of 8 Experiments
% Uc = [ 0.0;    0.0;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
% Ud = [[0.005; -0.005;  0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.01;   0.0;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%      [0.005;   0.005;  0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%     [-0.005;  -0.005;  0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%      [-0.01;   0.0;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%     [-0.005;   0.005;  0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1]];

% %Planning Zig Zag Experimeants
% Uc = [ 0.0;       0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
% Ud = [[0.0;       0.0;       0.0;        0.0;        0.0;        0.0;        0.0;       -0.1;    970.1;  970.1], ...
%       [0.0;       0.0;       0.0;        0.0;        0.0;        0.025;      0.0;       -0.075;    970.1;  970.1], ...
%       [0.0;       0.0;       0.0;        0.0;        0.0;       -0.075;      0.0;       -0.025;    970.1;  970.1], ...
%       [0.0;       0.0;       0.0;        0.0;        0.0;        0.1;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;       0.0;       0.0;        0.0;        0.0;       -0.075;      0.0;        0.025;    970.1;  970.1], ...
%       [0.0;       0.0;       0.0;        0.0;        0.0;        0.025;      0.0;        0.075;    970.1;  970.1], ...
%       [0.0;       0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.1;    970.1;  970.1]];

%Planning Bx By Experimeants
% Uc = [ 0.0;       0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1];    %Current Magnetic Field (Will be replaced with current position)
% Ud = [[0.0075;    0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;       0.0075;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%      [-0.0075;    0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0;      -0.0075;    0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1], ...
%       [0.0075;    0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.0;    970.1;  970.1]];
% 
% Xdes = [];

%% Field Experiments
% %Enter Desired End Effector Positions
% Xdes = [[-0.35;     -1.02;    0.19;    827.4953;   145.5150;  -485.0500;  0.35;    1.02;     0.19;   -827.4953;  -145.5150;  -485.0500], ...
%         [0.33;       0.0;     0.00;    970.1;      000.0;      000.0;    -0.33;    0.0;      0.0;    970.1;      000.0;      000.0], ...     %Bx 
%         [0.00;      -0.33;    0.00;    000.0;      970.1;      000.0;     0.00;    0.33;     0.0;    000.0;      970.1;      000.0], ...     %By
%         [0.18;      -0.18;    0.00;    000.0;      000.0;     -970.1;    -0.18;    0.18;     0.0;    000.0;      000.0;     -970.1], ...     %Bz
%         [0.195;      0.00;   -0.195;   686.1;      000.0;      685.8;    -0.195;   0.00;     0.195; -686.1;      000.0;     -685.8], ...     %dBxx
%         [0.275;      0.0;     0.00;    000.0;      970.1;      000.0;    -0.275;   0.0;      0.0;    000.0;     -970.1;      000.0], ...     %dBxy
%         [0.275;      0.0;     0.00;    000.0;      000.0;      970.1;    -0.275;   0.0;      0.00;   000.0;      000.0;     -970.1], ...     %dBxz
%         [0.000;     -0.195;   0.195;   3.515;     -683.7;     -688.2;     0.000;   0.195;   -0.195;  -4.376;      671.0;     700.6], ...     %dByy
%         [0.00;      -0.275;   0.00;    000.0;      000.0;     -970.1;     0.00;    0.275;    0.00;    000.0;      000.0;     970.1], ...     %dByz
%         [-0.35;     -1.02;    0.19;    827.4953;   145.5150;  -485.0500;  0.35;    1.02;     0.19;   -827.4953;  -145.5150;  -485.0500]];    %0         

%Constants for Field Finder
mu0 = 4*pi*1e-7;
mu = 970.1;
r_min = 0.1; %Minimum dist from magnet to origin
r_max = 1; %Maximum dist from magnet to origin
max_time = 3;


%Checking number of diff fields/positions to go to
if (isempty(Xdes) == 1 )
    [~, numFields] = size(Ud);
    U_request = cat(2,Uc,Ud); %Creating one matrix for all the Fields we're requestion
else
    [~, numFields] = size(Xdes);
    numFields = numFields -1; 
end

%Matrixes with the full Path for all requested fields. 
path_points = 10;
u_planFinal =  zeros([path_points*numFields, 6]);
X_planningFinal = zeros([path_points*numFields, 12]);
X_CSVFinal = zeros([path_points*numFields, 14]);
X_CSV_mu_Final = zeros([path_points*numFields, 12]);
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

        %Making sure robots are on correct side
        X1Cpos = Xc(1:3);
        X1Cmu = Xc(4:6);
        X2Cpos = Xc(7:9);
        X2Cmu = Xc(10:12);
        
        X1Dpos = Xd(1:3);
        X1Dmu = Xd(4:6);
        X2Dpos = Xd(7:9);
        X2Dmu = Xd(10:12);

        if(X1Cpos(2) >  0 && X2Cpos(2) < 0)
            %This should be robot 2
            Xc = [X2Cpos'  X2Cmu' X1Cpos' X1Cmu']';
        end

        if(X1Dpos(2) >  0 && X2Dpos(2) < 0)
            %This should be robot 2
            Xd = [X2Dpos'  X2Dmu' X1Dpos' X1Dmu']';
        end

    else
        Xc = Xdes(:,count);
        Uc = field(Xc);
        Xd = Xdes(:,count + 1);
        Ud = field(Xd);

        %Making sure robots are on correct side
        X1Cpos = Xc(1:3);
        X1Cmu = Xc(4:6);
        X2Cpos = Xc(7:9);
        X2Cmu = Xc(10:12);
        
        X1Dpos = Xd(1:3);
        X1Dmu = Xd(4:6);
        X2Dpos = Xd(7:9);
        X2Dmu = Xd(10:12);

        if(X1Cpos(2) >  0 && X2Cpos(2) < 0)
            %This should be robot 2
            Xc = [X2Cpos'  X2Cmu' X1Cpos' X1Cmu']';
        end

        if(X1Dpos(2) >  0 && X2Dpos(2) < 0)
            %This should be robot 2
            Xd = [X2Dpos'  X2Dmu' X1Dpos' X1Dmu']';
        end
    end


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
   
  
    %% Creating a Linear path in polar space
%     [X_planning, polarpath_1, polarpath_2] = pathCreate(Xc, Xd, path_points);

     %% Creating Multiple paths in polar space
    rhoDiff = 0.075; %Radius offset of the 3 diff paths created
    [X_planning, polarpathVarried_1, polarpathVarried_2] = pathCreateVarriedRho(Xc, Xd, path_points, rhoDiff);

        
    %% Analitical Solver for Pre Defined Path 
    %Creating a field path 
    U_path = zeros(8, path_points);
    
    %Linear field interpolation
%     for i = 1:8
%        U_path(i,:) = linspace(Uc(i), Ud(i), path_points);  %Creating a field path by linearly interpolating
%     end

    %Field path with a more exponential fit to field path instead of linear
    for i =  1:8
%         U_path(i,1) = Uc(i);
%         U_path(i,2) = Uc(i) .* 0.4;
%         U_path(i,3) = Uc(i) .* 0.1;
%         U_path(i,4) = Uc(i) .* 0.05;
%         U_path(i,5) = Uc(i) .* 0.02;
%         U_path(i,6) = Ud(i) .* 0.6;
%         U_path(i,7) = Ud(i) .* 0.9;
%         U_path(i,8) = Ud(i) .* 0.95;
%         U_path(i,9) = Ud(i) .* 0.975;
%         U_path(i,10) = Ud(i);        

        U_path(i,1) = Uc(i);
        U_path(i,2) = Uc(i) .* 0.9;
        U_path(i,3) = Uc(i) .* 0.6;
        U_path(i,4) = Uc(i) .* 0.2;
        U_path(i,5) = Uc(i) .* 0.05;
        U_path(i,6) = Ud(i) .* 0.1;
        U_path(i,7) = Ud(i) .* 0.4;
        U_path(i,8) = Ud(i) .* 0.8;
        U_path(i,9) = Ud(i) .* 0.95;
        U_path(i,10) = Ud(i);     
    end

    %Creating an array of which fields do not change
    U_noChange = zeros([8,1]);
    U_noChangeWeight = 0;
    for i = 1:8
        U_noChange(i) = ~any(Uc(i) - Ud(i));
    end
    double(U_noChange);

    %Solving for mew for each path and chosing one with lowwest error
    X_planningMewSolve = zeros(size(X_planning));
    u_plan = zeros(size(X_planning,1), 6, path_points);
    
    % Creating a Vector X to write to CSV
    X_CSV = zeros(size(X_planning,1), path_points,14);
    %Same Vector X but with mu instead of quaternions
    X_CSV_mu = zeros(size(X_planning,1), path_points,12);
    U_varriedPaths = zeros(size(X_planning,1), 10, path_points);

    for i = 1:size(X_planning,1)
        [u_plan(i,:,:), X_planningMewSolve(i,:,:), X_CSV(i,:,:), X_CSV_mu(i,:,:)] = analyticalMewSolve(Xc, Xd, squeeze(X_planning(i,:,:)), U_path);
        %Getting Fields for each path
        for a = 1:path_points
            U_varriedPaths(i,:,a) = field(squeeze(X_planningMewSolve(i,a,:)));
         end
    end

    %Calculating the Error at each point
    error_pathVarried = zeros(size(X_planningMewSolve,1), 8, path_points);
    normError_pathVarried = zeros(size(X_planningMewSolve,1), path_points);

    indexChosen = zeros(1,path_points);
    for i = 1:path_points
        %For each path
        for a = 1:size(X_planningMewSolve,1)
            error_pathVarried(a,:,i) = abs(U_varriedPaths(a,1:8,i) - U_path(:,i)');
%             %Weighting the elements which are not supposed to change more
%             for z = 1:8
%                 if (U_noChange(z) ~= 0)
%                     error_pathVarried(a,z,i) = error_pathVarried(a,z,i) .*U_noChangeWeight;
%                 end
%             end
%             %error_pathVarried(a,:,i) = error_pathVarried(a,:,i) .* (U_noChange.*U_noChangeWeight)';
            normError_pathVarried(a,i) = norm(error_pathVarried(a,:,i));
        end
        [~,indexChosen(i)] = min(normError_pathVarried(:,i));
        u_planFinal((i + (count-1)*path_points), :) =  squeeze(u_plan(indexChosen(i),:,i));
        X_planningFinal((i + (count-1)*path_points), :) = squeeze(X_planningMewSolve(indexChosen(i),i,:));
        X_CSVFinal((i + (count-1)*path_points), :) = squeeze(X_CSV(indexChosen(i),i,:));
        X_CSV_mu_Final((i + (count-1)*path_points), :) = squeeze(X_CSV_mu(indexChosen(i),i,:));

        polarpath_1(i,:) = squeeze(polarpathVarried_1(indexChosen(i),i,:));
        polarpath_2(i,:) = squeeze(polarpathVarried_2(indexChosen(i),i,:));

    end

    Uc_plot = repmat(Uc, 1, path_points);
    Ud_plot = repmat(Ud, 1, path_points);
    Uc_plotFinal(:, (path_points*(count-1) + 1):(path_points*count)) = Uc_plot;
    Ud_plotFinal(:, (path_points*(count-1) + 1):(path_points*count)) = Ud_plot;
    U_pathFinal(:, (path_points*(count-1) + 1):(path_points*count)) = U_path;
    
 
%      %Plotting Fields for each path
%     figure();
%     for a = 1:3
%         for i = 1:8
%             subplot(4, 2, i)
%                 grid on;
%                 if a ==1
%                     plot(1:10, U_path(i,:), '--k', 'LineWidth', 1.0)
%                     hold on;
%                 end
%                 
%                 plot(1:10, squeeze(U_varriedPaths(a, i,:)), 'LineWidth', 1.0)
%                 hold on
%     
%         
%                 xlabel('Points in Path (s)', 'FontSize', 14)
%                 ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
%         
%         %         if i <= 3
%         %             ylim([-0.01 0.01]);
%         %         else
%         %             ylim([-0.1 0.1]);
%         %         end
%         % 
%                 if i == 1
%                     legend('Desired','Original - rho', 'Original', 'Original + rho', 'FontSize', 12)
%                 end        
%         end
%     end
%     sgtitle("Fields for Each Path", 'FontSize', 24)
% 
%     %Plotting Errors for each path
%     figure();
%     for a = 1:3
%         for i = 1:8
%             subplot(4, 2, i)
%             grid on;
% 
%             plot(1:10, squeeze(error_pathVarried(a, i,:)), 'LineWidth', 1)
%             hold on
% 
%             xlabel('Points in Path (s)', 'FontSize', 14)
%             ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
%     
%             if i == 1
%                 legend('Original - rho', 'Original', 'Original + rho', 'FontSize', 12)
%             end        
%         end
%     end
%     sgtitle("Errors for Each Path", 'FontSize', 24)
% 
%     figure();
%     
%     for i = 1:3
%         plot(1:10, normError_pathVarried(i,:))
%         grid on
%         hold on;
%     end
%     legend('Original - rho', 'Original', 'Original + rho', 'FontSize', 12)
%     title("Norm Errors for Each Path", 'FontSize', 24)

     %% Plotting Path Chosen
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

end




%% Plotting Field at each point in path
U_final = zeros(10, path_points*count);

for a = 1:path_points*count
    U_final(:,a) = field(X_planningFinal(a,:));
end

figure();
%fig = figure('Units','centimeters','Position',[10 3 10.5 7.4]);
for i = 1:8
    subplot(4, 2, i)
        plot(1:path_points*count, U_pathFinal(i, :)', '-', 'LineWidth', 3.0, 'Color', '#0072BD')
        hold on
%         plot(1:path_points*count, Uc_plotFinal(i, :)', 'ro', 'LineWidth', 1.0)
%         hold on
%         plot(1:path_points*count, Ud_plotFinal(i, :)', 'go', 'LineWidth', 1.0)
        hold on
        plot(1:path_points*count, U_final(i,:), ':', 'LineWidth', 4.0, 'Color', '#FF0000')
        grid on
        xlabel('Points in Path', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        %ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)

        if i == 1
            ylabel(strcat('$B_x $ (mT)'), 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        elseif i == 2
            ylabel(strcat('$B_y $ (mT)'), 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        elseif i == 3
            ylabel(strcat('$B_z $ (mT)'), 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        elseif i == 4
            ylabel(strcat('$dB_x /{dx} $ (mT/m)'), 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        elseif i == 5
            ylabel(strcat('$dB_x /{dy} $ (mT/m)'), 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        elseif i == 6
            ylabel(strcat('$dB_x /{dz} $ (mT/m)'), 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        elseif i == 7
            ylabel(strcat('$dB_y /{dy} $ (mT/m)'), 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        elseif i == 8
            ylabel(strcat('$dB_y /{dz} $ (mT/m)'), 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        end

        if i <= 3
            ylim([-0.02 0.02]);
        else
            ylim([-0.2 0.2]);
        end

%         if i == 1
%             legend('Path', 'Start', 'Desired', 'Path Followed', 'FontSize', 12)
%         end     

        if i == 1
            legend('$U_{Path}$', '$U_{Planner}$', 'Interpreter', 'latex', 'FontSize', 14, 'FontName', 'TimesNewRoman')
        end  
end
%sgtitle("Planned Field for Full Path", 'FontSize', 24)



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

% %% Plotting Wrenches
% figure();
% for i = 1:6
%     subplot(3, 2, i)
%         plot(1:path_points*count, w1_plan(i, :)', 'k', 'LineWidth', 1.0)
%         hold on
%         plot(1:path_points*count, w1_des(i, :)', 'r--', 'LineWidth', 2.0)
%         xlabel('Points in Path (s)', 'FontSize', 14)
% 
%         if i < 4
%             ylabel(strcat('$\tau_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
%         else
%             ylabel(strcat('$f_', num2str(i-3),'$'), 'Interpreter', 'latex', 'FontSize', 14)
%         end
%        
% 
%         grid on;
% 
%         if i == 1
%             legend('Wrench', 'Desired', 'FontSize', 12)
%         end        
% 
%         if i <= 3
%             ylim([-0.03 0.03]);
%         else
%             ylim([-0.15 0.15]);
%         end
% 
%         %Setting plot bakground to grey for axsis we cant control
%         if i == 3
%             set(gca,'color',[0.6235, 0.6235, 0.6235])
%         end
% end
% sgtitle("Planned Wrench w1 for Full Path", 'FontSize', 24)
% 
% figure();
% for i = 1:6
%     subplot(3, 2, i)
%         plot(1:path_points*count, w2_plan(i, :)', 'k', 'LineWidth', 1.0)
%         hold on
%         plot(1:path_points*count, w2_des(i, :)', 'r--', 'LineWidth', 2.0)
%         xlabel('Points in Path (s)', 'FontSize', 14)
% 
%         if i < 4
%             ylabel(strcat('$\tau_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
%         else
%             ylabel(strcat('$f_', num2str(i-3),'$'), 'Interpreter', 'latex', 'FontSize', 14)
%         end
%       
%         grid on;
% 
%         if i == 1
%             legend('Wrench', 'Desired', 'FontSize', 12)
%         end        
% 
%         if i <= 3
%             ylim([-0.03 0.03]);
%         else
%             ylim([-0.15 0.15]);
%         end
% 
%         %Setting plot bakground to grey for axsis we cant control
%         if i == 2 || i == 3 || i == 6
%             set(gca,'color',[0.6235, 0.6235, 0.6235])
%         end
% end
% sgtitle("Planned Wrench w2 for Full Path", 'FontSize', 24)

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

% %  ---- Wrenches ----
% ErrWrench1 = zeros([size(w1_plan)]);
% ErrWrench2 = zeros([size(w2_plan)]);
% for i = 1:6
%     ErrWrench1(i,:) = abs((w1_plan(i,:) - w1_des(i,:))); 
%     ErrWrench2(i,:) = abs((w2_plan(i,:) - w2_des(i,:))); 
% end
% %norm Torque W1 Error
% NormWrench1(1,:) = vecnorm(ErrWrench1(1:3,:));
% %norm Force W1 Error
% NormWrench1(2,:) = vecnorm(ErrWrench1(4:6,:));
% 
% %norm Torque W2 Error
% NormWrench2(1,:) = vecnorm(ErrWrench2(1:3,:));
% %norm Force W2 Error
% NormWrench2(2,:) = vecnorm(ErrWrench2(4:6,:));
% 
% %Plotting Errors for Load Cell 1
% figure();
% for i = 1:8
%     subplot(5, 2, i)
%     if i < 7
%         plot(1:path_points*count, ErrWrench1(i, :)', 'r', 'LineWidth', 1.0)
%         xlabel('Points in Path (s)', 'FontSize', 14)
%         ylabel(strcat('$ERROR \omega_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
%     elseif i == 7
%         plot(1:path_points*count, NormWrench1(i-6, :)', 'k', 'LineWidth', 1.0)
%         title('Norm Torque Error', 'FontSize', 18)
%         xlabel('Points in Path (s)', 'FontSize', 14)
%         ylabel(strcat('$Norm \tau','$'), 'Interpreter', 'latex', 'FontSize', 14)
%     else
%         plot(1:path_points*count, NormWrench1(i-6, :)', 'k', 'LineWidth', 1.0)
%         title('Norm Force Error', 'FontSize', 18)
%         xlabel('Points in Path (s)', 'FontSize', 14)
%         ylabel(strcat('$Norm f','$'), 'Interpreter', 'latex', 'FontSize', 14)
%     end
% 
%     %Setting plot bakground to grey for axsis we cant control
%     if i == 3
%         set(gca,'color',[0.6235, 0.6235, 0.6235])
%     end
%      
% end
% sgtitle("Error for Load Cell 1", 'FontSize', 24)
% 
% 
% %Plotting Errors for Load Cell 2
% figure();
% for i = 1:8
%     subplot(5, 2, i)
%     if i < 7
%         plot(1:path_points*count, ErrWrench2(i, :)', 'r', 'LineWidth', 1.0)
%         xlabel('Points in Path (s)', 'FontSize', 14)
%         ylabel(strcat('$ERROR \omega_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
%     elseif i == 7
%         plot(1:path_points*count, NormWrench2(i-6, :)', 'k', 'LineWidth', 1.0)
%         title('Norm Torque Error', 'FontSize', 18)
%         xlabel('Points in Path (s)', 'FontSize', 14)
%         ylabel(strcat('$Norm \tau','$'), 'Interpreter', 'latex', 'FontSize', 14)
%     else
%         plot(1:path_points*count, NormWrench2(i-6, :)', 'k', 'LineWidth', 1.0)
%         title('Norm Force Error', 'FontSize', 18)
%         xlabel('Points in Path (s)', 'FontSize', 14)
%         ylabel(strcat('$Norm f','$'), 'Interpreter', 'latex', 'FontSize', 14)
%     end
% 
%     %Setting plot bakground to grey for axsis we cant control
%     if i == 2 || i == 3 || i == 6
%         set(gca,'color',[0.6235, 0.6235, 0.6235])
%     end
%      
% end
% sgtitle("Error for Load Cell 2", 'FontSize', 24)