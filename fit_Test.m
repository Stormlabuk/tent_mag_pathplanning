%% Figuring out the fit for the fit used for the path planner
close all
clear all

%% Values used in path planner
Init_Val = -10;
Des_Val = 1;

U_Path_Decay = [1, 0.9, 0.6, 0.2, 0.005];
U_Path_Growth = [0.1, 0.4, 0.8, 0.95, 1];

% U_path(i,1) = Uc(i);
%         U_path(i,2) = Uc(i) .* 0.9;
%         U_path(i,3) = Uc(i) .* 0.6;
%         U_path(i,4) = Uc(i) .* 0.2;
%         U_path(i,5) = Uc(i) .* 0.05;
%         U_path(i,6) = Ud(i) .* 0.1;
%         U_path(i,7) = Ud(i) .* 0.4;
%         U_path(i,8) = Ud(i) .* 0.8;
%         U_path(i,9) = Ud(i) .* 0.95;
%         U_path(i,10) = Ud(i);     

for i = 1:10
    if i < 6
        Path(i) = Init_Val * U_Path_Decay(i);
    else
        Path(i) = Des_Val * U_Path_Growth(i-5);
    end
end

%% Finding expo fit for first half
MathPath = [];
MathPath(1) = Init_Val;
for i = 1:5
    MathPath(i+1) = Init_Val/(1 + exp(2.1*(i-2.2)));
end

%Growth for second half
for i = 1:5
    MathPath(i+5) = Des_Val/(1 + exp(-2.1*(i-2.2)));
end
%MathPath(10) = Des_Val;

%% Plotting 
figure
plot(Path);
hold on
yline(Init_Val, 'r');
hold on 
yline(Des_Val, 'g')
ylim([-12 12]);
hold on
xline(5, '--k')

%Plotting Equation Graph 
hold on
plot(MathPath, 'r');