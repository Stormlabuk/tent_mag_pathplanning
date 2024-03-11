%% Using a premade joint planner so that there are no abrupt changes in robot configuration 
close all

%Adding paths
addpath(genpath('urdf'));
addpath(genpath('meshes_kuka_iiwa'));

%importing Robot URDFs
iiwa_1 = importrobot('/urdf/kuka_iiwa_1.urdf'); % 14 kg payload version
iiwa_1.DataFormat = 'row';
iiwa_2 = importrobot('/urdf/kuka_iiwa_2.urdf'); % 14 kg payload version
iiwa_2.DataFormat = 'row';

figure
show(iiwa_1)
hold on
show(iiwa_2)
zlim([ 1.5]);