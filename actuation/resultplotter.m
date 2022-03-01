%%Figure Plotter
close all
clear all

load('resultsFP_JD.mat')

% %plotting desired vs achieved for each gradient 
% for a = 1:8
% 
%     figure(1)
%         subplot(4, 2, a)
%         plot(1:testitt,U_request(a,:), 'b')
%         hold on 
%         scatter(1:testitt,U_recieved(a,:),'r')
%         xlabel('Sample Number');
%         ylabel(strcat('$U_', num2str(a), '$'), 'Interpreter', 'latex');
%         legend('Desired','Achieved')
%         grid on;
% end

% %plotting desired vs achieved for each gradient 
for a = 1:8

    figure(1)
    %title("No FP Shift")
    subplot(4, 2, a)
    plot(1:itt,U_request(a,:), 'b')
    hold on 
    scatter(1:itt,UFinal(a,:),'r')
    xlabel('Sample Number');
    ylabel(strcat('$U_', num2str(a), '$'), 'Interpreter', 'latex');
    legend('Desired','Achieved')
    grid on;
end

figure(2)
% plot(normErr)
% grid on;
% xlabel('Sample Number')
% ylabel('Error')

title("Error")
plot(1:itt,Uerr, 'b');
hold on;
plot(1:itt, UerrFP, 'r');
hold on;
plot(1:itt, errFinal, 'g');
legend('No FP Shift', 'FP Shift', 'Final');

%% Checking if Resutls for Position of Magnets make sense
% nanErr = 0;
% muErr = 0;
% xbigErr = 0;
% xsmallErr = 0;
% mag2closeErr = 0;
% totErr = 0;
% for i = 1:length(Xfinal)
%     if isnan(Xfinal(:,i)) == 1
%         nanErr = nanErr + 1;
%     elseif norm(Xfinal(1:3,i) - Xfinal(7:9,i)) < 0.1
%         mag2closeErr = mag2closeErr + 1;
%     elseif norm(Xfinal(1:3,i)) > 0.75 || norm(Xfinal(7:9,i)) > 0.75 
%         xbigErr = xbigErr + 1;
%     elseif norm(Xfinal(1:3,i)) < 0.15 || norm(Xfinal(7:9,i)) < 0.15 
%         xsmallErr = xsmallErr + 1;
%     elseif norm(Xfinal(4:6,i)) > 971.1 || norm(Xfinal(10:12,i)) > 971.1
%         muErr = muErr + 1;
%     elseif norm(Xfinal(4:6,i)) < 969.1 || norm(Xfinal(10:12,i)) < 969.1 
%         muErr = muErr + 1;
%     end
%     
% end
% 
% totErr = nanErr + muErr + xbigErr + xsmallErr + mag2closeErr; 
% 
% disp('Error is: ')
% disp(totErr/testitt * 100)

%% Caclulating Errors
totErr = 0;

for i = 1:itt
    if isnan(XFinal(:,i)) == 1
        nanErr = nanErr + 1;
    elseif norm(XFinal(1:3,i) - XFinal(7:9,i)) < 0.1
        mag2closeErr = mag2closeErr + 1;
    elseif norm(XFinal(1:3,i)) < 0.1 || norm(XFinal(7:9,i)) < 0.1    
        fpOutErr = fpOutErr + 1;
    elseif norm(XFinal(4:6,i)) > 971.1 || norm(XFinal(10:12,i)) > 971.1
        muErr = muErr + 1;
    elseif norm(XFinal(4:6,i)) < 969.1 || norm(XFinal(10:12,i)) < 969.1 
        muErr = muErr + 1;
    end
    
end

totErr = nanErr + muErr + fpOutErr + mag2closeErr; 

disp('Error is: ')
disp(totErr/itt * 100)

count = 1;
for i = 1:itt
    if isnan(errFinal(i)) == 0
        errPlot(count) = errFinal(i);
        count = count + 1;
    else
        count = count + 1;
    end
    
end

