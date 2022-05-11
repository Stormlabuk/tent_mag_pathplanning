%% TEST SCRIPT
clear all
close all

load census;
f=fit(cdate,pop,'poly2')
figure
plot(f,cdate,pop)