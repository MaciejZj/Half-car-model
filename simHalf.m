clear;

SuspensionParams.m_1 = 20;     % [kg]
SuspensionParams.k_1 = 200000; % [N/m]
SuspensionParams.C_1 = 200;    % [N*m/s]

SuspensionParams.m_2 = 20;     % [kg]
SuspensionParams.k_2 = 200000; % [N/m]
SuspensionParams.C_2 = 200;    % [N*m/s]

SuspensionParams.m_s = 400;    % [kg]
SuspensionParams.k_3 = 30000;  % [N/m]
SuspensionParams.C_3 = 1000;   % [N*m/s]
SuspensionParams.k_4 = 30000;  % [N/m]
SuspensionParams.C_4 = 1000;   % [N*m/s]

SuspensionParams.I_s = 50;     % [kg*m^2]
SuspensionParams.l_F = 1.5;    % [m]
SuspensionParams.l_R = 1;      % [m]

SimulationParams.SuspensionParams = SuspensionParams;
SimulationParams.actuationTime = 3; % [s]
SimulationParams.actuationVals = [1, 1, 1, 1];
SimulationParams.actuationFun = @ramp;

X0 = [0.01, 0, 0, 0, 0, 0, 0, 0];
tRange = 0 : 0.01 : 10;

modelFun = @(t,y) (modelHalf(t, y, SimulationParams));
[ T, X ] = ode45(modelFun, tRange, X0, odeset('RelTol', 1e-5, 'AbsTol', 1e-5));

for i = 1 : 8
    if mod(i, 2) == 0
       plotTitle = "Velocity";
       plotyLabel = "velocity [m/s]";
    else
       plotTitle = "Postition";
       plotyLabel = "position [m]";
    end
    
    figure(i);
    plot(T, X(:,i));
    title(strcat(plotTitle, " ", num2str(ceil(i/2))));
    ylabel(strcat(plotyLabel));
end

csvwrite('dataMat.csv', [T, X])

function Z_0 = ramp(actuationVals, t, actuationTime)
    if (t < actuationTime)
        Z_0(1) = 0;
        Z_0(2) = 0;
        Z_0(3) = 0;
        Z_0(4) = 0;
    elseif (t == actuationTime)
        Z_0(1) = actuationVals(1) * (t - actuationTime);
        Z_0(2) = actuationVals(2) * 1;
        Z_0(3) = actuationVals(3) * (t - actuationTime);
        Z_0(4) = actuationVals(4) * 1;
    else
        Z_0(1) = actuationVals(1) * (t - actuationTime);
        Z_0(2) = actuationVals(2) * 1;
        Z_0(3) = actuationVals(3) * (t - actuationTime);
        Z_0(4) = actuationVals(4) * 1;
    end
end

function Z_0 = step(actuationVals, t, actuationTime)
    if (t < actuationTime)
        Z_0(1) = 0;
        Z_0(2) = 0;
        Z_0(3) = 0;
        Z_0(4) = 0;
    elseif (t == actuationTime)
        Z_0(1) = actuationVals(1);
        Z_0(2) = inf;
        Z_0(3) = actuationVals(3);
        Z_0(4) = inf;
    else
        Z_0(1) = actuationVals(1);
        Z_0(2) = 0;
        Z_0(3) = actuationVals(3);
        Z_0(4) = 0;
    end
end