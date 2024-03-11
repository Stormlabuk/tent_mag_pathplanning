%% CSV Plotter
close all
clear all
 
%% Paths of CSVs to Read
pathPlanning = 'C:\Users\elmbr\OneDrive - University of Leeds\Publications\MagneticPlanner\CSVs\Fields\Results\SensorDataAllFieldsPlanning.csv';
pathNoPlanning = 'C:\Users\elmbr\OneDrive - University of Leeds\Publications\MagneticPlanner\CSVs\Fields\Results\SensorDataAllFieldsNoPlanning.csv';
pathDesiredFields = 'C:\Users\elmbr\OneDrive - University of Leeds\Publications\MagneticPlanner\CSVs\Fields\Results\DesiredFields.csv';

%Importing CSVs
planningFields = csvread(pathPlanning,1);  % skips the first row of data
noPlanningFields = csvread(pathNoPlanning,1);
desFields = csvread(pathDesiredFields,1);

%Chaning from table to Variable
%rows2vars(planningFields)

%% Plotting
[plotxPlanLen, ~] = size(planningFields);
[plotxDesLen, ~] = size(desFields);
[plotxNoPlanLen, ~] = size(noPlanningFields);

plotxPlan = 1:plotxPlanLen;
plotxDes = linspace(1,plotxPlanLen, plotxDesLen);
plotxNoPlan = linspace(1,plotxPlanLen, plotxNoPlanLen);

figure();
for i = 1:8
    subplot(4, 2, i)

        plot(plotxDes, desFields(:, i)', 'k', 'LineWidth', 1.0)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
        hold on

        plot(plotxPlan, planningFields(:, i)', 'b', 'LineWidth', 1.0)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
        hold on
        
        plot(plotxNoPlan, noPlanningFields(:, i)', 'r', 'LineWidth', 1.0)
        xlabel('Points in Path (s)', 'FontSize', 14)
        ylabel(strcat('$U_', num2str(i),'$'), 'Interpreter', 'latex', 'FontSize', 14)
        grid on
        hold on

        

        if i <= 3
            ylim([-0.02 0.02]);
        else
            ylim([-0.2 0.2]);
        end

        if i == 1
            legend('Desired', 'Planning', 'NoPlanning', 'FontSize', 12)
        end        
end
sgtitle("Planned Field for Full Path", 'FontSize', 24)
