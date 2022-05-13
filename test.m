%% TEST SCRIPT
clear all
close all

Ud = [[0.0;     0.0;       0.0;        0.0;        0.0;        0.1;        0.0;        0.0;    970.1;  970.1], ...
      [0.0;     0.0;       0.0;        0.0;        0.0;        0.0;        0.0;        0.1;    970.1;  970.1]];

x = [1, 2, 3, 4, 5, 6, 7]';
%y = [0,0.002,0.004, 0.006, 0.095, 0.1]';
Ubetween(:,1) = Ud(1:8,1);
Ubetween(:,2) = Ud(1:8,1)*0.4;
Ubetween(:,3) = Ud(1:8,1)*0.1;
Ubetween(:,4) = Ud(1:8,1)*0.05;
Ubetween(:,5) = Ud(1:8,end)*0.6;
Ubetween(:,6) = Ud(1:8,end)*0.9;
Ubetween(:,7) = Ud(1:8,end);

for i = 1:8
    Ufit{i,:}=fit(x,Ubetween(i,:)','exp1');
end

%% Exponential Plots
x = [1:10];

for i = 2:10
    y(i) = x(i-1)*(1 - exp(-(i/100)/500));
end
figure
plot(x,y,'o')

%% Plotting Field at each point in path
figure();
for i = 1:8
    subplot(4, 2, i)
        plot(x, Ubetween(i, :)', 'o', 'LineWidth', 1.0)
        hold on
        plot(Ufit{i})

        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)

%         if i <= 3
%             ylim([-0.01 0.01]);
%         else
%             ylim([-0.1 0.1]);
%         end
% 
%         if i == 1
%             legend('Path', 'Start', 'Desired', 'Path Followed', 'FontSize', 12)
%         end        
end
sgtitle("Planned Field for Full Path", 'FontSize', 24)