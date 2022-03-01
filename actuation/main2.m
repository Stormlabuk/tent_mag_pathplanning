% Field search with multiple EPMs
clear
close all
clc

rmpath(genpath('functions'))
addpath(genpath('functions'))

U = [0.01*[0; 0; 0];
    0.1*[0; 0; 0; 0; 1]]

X = fieldSearch(U, eps);

Uc = field_2(4*pi*1e-7, X)