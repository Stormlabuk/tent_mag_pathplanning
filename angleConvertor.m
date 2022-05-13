%% Angle Converstion Script

%Rotations in robot (old) frame
p = [0.21	0.21 	0.0];
q = [0	-0.7071	0	0.7071];
R = quat2rotm(q);

%Need to convet to sensor frame, which is rotz(45)
rot45 = rotz(45);
pnew  = rot45*p'
Rnew = rot45 * R

mu = Rnew * [1,0,0]'%(p./norm(p))'
mu = mu.*970.1