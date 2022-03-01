clear
close all
clc

rmpath(genpath('functions'))
addpath(genpath('functions'))


t = -pi/4;
r1 = [cos(t); 0; sin(t)];
r2 = [sin(t); 0; -cos(t)];

id_6 = eye(6);

dU_dm = [];
for i = 1:6
    m1 = id_6(1:3, i);
    m2 = id_6(4:6, i);
    X = [r1; m1; r2; m2];

    dU_dm = [dU_dm, field_2(1, X)];
end

id_8 = eye(8);
err = [];
res = [];
for i = 1:8
    res = [res, dU_dm\id_8(:, i)];
    m1 = res(1:3, i);
    m2 = res(4:6, i);
    X = [r1; m1; r2; m2];
    err = [err, field_2(1, X)];
end
res
err