%% Change from Qauternions to Mew

X = [0.069225871	-0.285711644	-6.26E-18	0.904358487	1.10E-17	2.66E-17	-0.426773624	-0.069225871	0.285711644	6.26E-18	0.904358487	1.10E-17	2.66E-17	-0.426773624];
X_mew = zeros([1,12]);

%Keeping same positions
X_mew(1:3) = X(1:3);
X_mew(7:9) = X(8:10);

%Rot1
rot1 = quat2rotm(X(4:7));
mew1 = rot1*[1;0;0];

X_mew(4:6) = mew1.*970.1;

%Rot2
rot2 = quat2rotm(X(11:14));
mew2 = rot2*[1;0;0];

X_mew(10:12) = mew2.*970.1;