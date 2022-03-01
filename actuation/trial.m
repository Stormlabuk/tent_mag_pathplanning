clear
close all
clc

rmpath(genpath('functions'))
addpath(genpath('functions'))


id_6 = eye(6);
id_8 = eye(8);

ERR = [];

count = 1;
for t = -2*pi:pi/100:2*pi
    
    r1 = [cos(t); 0; sin(t)];
    r2 = [sin(t); 0; -cos(t)];
    
    dU_dm = [];
    for i = 1:6
        m1 = id_6(1:3, i);
        m2 = id_6(4:6, i);
        X = [r1; m1; r2; m2];

        dU_dm = [dU_dm, field_2(1, X)];
    end
    
    err = [];
    for i = 4:8
        res = dU_dm\id_8(:, i);
        m1 = res(1:3);
        m2 = res(4:6);
        X = [r1; m1; r2; m2];
        err = [err, norm(id_8(:, i) - field_2(1, X))];
    end
    ERR = [ERR;
        err];
    count = count + 1;
end
