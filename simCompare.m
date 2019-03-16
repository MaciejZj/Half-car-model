X_Gcc = csvread('dataGcc.csv');
X_Mat = csvread('dataMat.csv');

for i = 2 : 8
    if mod(i, 2) == 0
       plotTitle = "Velocity";
       plotyLabel = "velocity [m/s]";
    else
       plotTitle = "Postition";
       plotyLabel = "position [m]";
    end
    
    figure(i);
    plot(X_Gcc(:,1), X_Gcc(:,i));
    hold on;
    plot(X_Mat(:,1), X_Mat(:,i));
    hold off;
    legend('Gsl based simulation', 'Matlab script based simulation');
    title(strcat(plotTitle, " ", num2str(ceil(i/2))));
    ylabel(strcat(plotyLabel));
end