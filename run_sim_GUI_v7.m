%% ENGR6014: Motorsport Vehicle Performance 2025-2026
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.0

clear; clc; close all;

% Find the middle of the screen

screensize = get(0, 'ScreenSize');
WindowWidth = 900;
WindowHeight = 1100;
leftpos = (screensize(3) - WindowWidth) / 2;
bottompos = (screensize(4) - WindowHeight) / 2;

Window = uifigure('Name', 'Lap Time Simulator', 'NumberTitle', 'off', 'Position', [leftpos, bottompos, WindowWidth, WindowHeight], 'Color', [0.94 0.94 0.94], 'Scrollable', 'on');

% Column for the vehicle Parameters

uilabel(Window, 'text', 'Vehicle Parameters', 'Position', [10 1080 200 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

vehicle_data = {'Vehicle Mass', 'Weight Dist Front', 'Wheelbase', 'Track Width F', 'Track Width R', 'CG Height', 'Roll Centre F', 'Roll Centre R', 'K Spring F', 'K Spring R', 'Motion Ratio F', 'Motion Ratio R', 'ARB Wheel Rate F', 'ARB Wheel Rate R','Aero Balance front', 'Aero Drag Factor', 'Aero Downforce factor', 'Final drive ratio', 'Gear Ratio', 'Final driveline efficiency', 'Motor RPM', 'Motor Torque'};

default_values = {'810', '0.45', '3020', '1500', '1500', '300', '8', '40', '175', '150', '1.18', '1.15', '260', '30','0.43', '0.50', '0.70', '9.40', '1.00', '0.88', '0 2000 4000 6000 8000 10000 12000 14000 16000 18000', '360 360 360 360 270 216 180 154 135 120'};

value_units = {'kg', 'fraction', 'mm', 'mm', 'mm', 'mm', 'mm', 'mm', 'N/mm', 'N/mm', 'Value', 'Value', 'N/mm', 'N/mm', 'Fraction', 'Kg/m', 'Kg/m', 'Ratio', 'Ratio', 'Fraction', 'RPM', 'Nm'};

n = length(vehicle_data);
input_box = gobjects(n, 1);
track_input_box = gobjects(n, 1);
Ftyre_input_box = gobjects(n, 1);
Rtyre_input_box = gobjects(n, 1);

for i = 1:n
    y = 1050 - (i-1)*28;
    uilabel(Window, 'text', vehicle_data{i}, 'Position', [10,y,175,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
    input_box(i) = uieditfield(Window, 'text', 'Value', default_values{i}, 'Editable', 'on', 'Position', [190,y+5,75,20], 'FontSize', 12, 'BackgroundColor', 'white', 'FontColor', 'black');
    uilabel(Window, 'text',  value_units{i}, 'Position', [270,y,60,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
end

% Circuit parameters: 

uilabel(Window, 'text', 'Track Parameters', 'Position', [350 1080 200 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

track_data = {'Corner Radius', 'Corner Angle', 'Length Straight'};

track_default_values = {'50 15 35 70', '90 151 38 81', '742 405 368 53'};

track_units = {'m', 'Deg', 'm'};

l = length(track_data);

for b = 1:l
    y = 1050 - (b-1)*28;
    uilabel(Window, 'text', track_data{b}, 'Position', [350 y 200 25], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
    track_input_box(b) = uieditfield(Window, 'Text', 'Value', track_default_values{b}, 'Position', [530,y+5,125,20], 'FontSize', 12, 'BackgroundColor', 'white', 'FontColor', 'black');
    uilabel(Window, 'text', track_units{b}, 'Position', [660,y,30,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
end

% Tyre Parameters:

y0 = 1050 - (i-1)*28;
yd = y0 - 35;

uilabel(Window, 'text', 'Front Tyre Parameters', 'Position', [10 yd 200 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

Ftyre_data = {'Rolling Radius', 'Decay Coefficient', 'Tyre Width', 'Longitudinal peak Mu', 'Lateral Peak Mu', 'Tyre Spring Rate'};

Ftyre_default_values = {'340', '3e-5', '260', '1.30', '1.36', '210'};

Ftyre_units = {'mm', 'Value', 'mm', 'Value', 'Value', 'N/mm'};

s = length(Ftyre_data);

for c = 1:s
    y = (yd-30) - (c - 1)* 28;
    uilabel(Window, 'text', Ftyre_data{c}, 'Position', [10 y 200 25], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
    Ftyre_input_box(c) = uieditfield(Window, 'Text', 'Value', Ftyre_default_values{c}, 'Position', [190,y+5,75,20], 'FontSize', 12, 'BackgroundColor', 'white', 'FontColor', 'black');
    uilabel(Window, 'text', Ftyre_units{c}, 'Position', [270,y,30,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
end

ye = (yd-30) - (s - 1)* 28;
yf = ye -35;

uilabel(Window, 'text', 'Rear Tyre Parameters', 'Position', [10 yf 200 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

Rtyre_data = {'Rolling Radius', 'Decay Coefficient', 'Tyre Width', 'Longitudinal peak Mu', 'Lateral Peak Mu', 'Tyre Spring Rate'};

Rtyre_default_values = {'340', '3e-5', '380', '1.30', '1.36', '245'};

Rtyre_units = {'mm', 'Value', 'mm', 'Value', 'Value', 'N/mm'};

w = length(Rtyre_data);

for d = 1:w
    y = (yf-30) - (d - 1)* 28;
    uilabel(Window, 'text', Rtyre_data{d}, 'Position', [10 y 200 25], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
    Rtyre_input_box(d) = uieditfield(Window, 'Text', 'Value', Rtyre_default_values{d}, 'Position', [190,y+5,75,20], 'FontSize', 12, 'BackgroundColor', 'white', 'FontColor', 'black');
    uilabel(Window, 'text', Rtyre_units{d}, 'Position', [270,y,30,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
end

%Simulation parameters:
Delta_S             = 0.1;   % Calculation step size interval [m]

% Axes placeholder in Tab visualisation
axHeight = 180; 
axWidth = 460; 
axLeft = 360;

parameterCount = 3;
parameterCount2 = 1;
parameterCount3 = 2;

tabVis = uitabgroup(Window, 'Position', [350 330 490 650]);

tab1 = uitab(tabVis, 'Title', 'Lap Time');
tab2 = uitab(tabVis, 'Title', 'Tyre Friction');
tab3 = uitab(tabVis, 'Title', 'Load Transfer');
tab4 = uitab(tabVis, 'Title', 'Documentation');

lapTime_label = uilabel(tab1, 'text', 'Lap time: ', 'Position', [100 590 250 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

hAxes_laptime = gobjects(parameterCount, 1);
hAxes_tyre_fric = gobjects(parameterCount2, 1);
hAxes_LoadTrans = gobjects(parameterCount3, 1);

for ii = 1:parameterCount 
    axBottom = 10 + (parameterCount - ii) * (axHeight + 20); 
    hAxes_laptime(ii) = uiaxes(tab1, 'Position', [10, axBottom, axWidth, axHeight]); 
end

for iii = 1:parameterCount3 
    axBottom = 10 + (parameterCount3 - iii) * (axHeight + 20); 
    hAxes_LoadTrans(iii) = uiaxes(tab3, 'Position', [10, axBottom + 200, axWidth, axHeight]); 
end

hAxes_tyre_fric(1) = uiaxes(tab2, 'Position', [10, 300, axWidth, 300]);

% Control buttons

uibutton(Window, 'Text', 'START', 'Position', [720,1040,120,35], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'white', 'ButtonPushedFcn', @(src,evt) calculate(src, evt, input_box, track_input_box, Ftyre_input_box, Rtyre_input_box, Delta_S, hAxes_laptime, hAxes_tyre_fric, hAxes_LoadTrans, lapTime_label));

uibutton(Window, 'Text', 'RESET', 'Position', [720,1000,120,35], 'FontSize', 10, 'BackgroundColor', [0.7 0.2 0.2], 'FontColor', 'white', 'ButtonPushedFcn', @(src,evt) reset(src, evt, n, l, s, w, parameterCount, input_box, track_input_box, Ftyre_input_box, Rtyre_input_box, default_values, track_default_values, Ftyre_default_values, Rtyre_default_values, hAxes_laptime, hAxes_tyre_fric, lapTime_label, hAxes_LoadTrans));

function reset(~,~, n, l, s, w, parameterCount, input_box, track_input_box, Ftyre_input_box, Rtyre_input_box, default_values, track_default_values, Ftyre_default_values, Rtyre_default_values, hAxes_laptime, hAxes_tyre_fric, lapTime_label, hAxes_LoadTrans)
    for k = 1:n
        input_box(k).Value = default_values{k};
    end
    for k = 1:l
        track_input_box(k).Value = track_default_values{k};
    end
    for k = 1:s
        Ftyre_input_box(k).Value = Ftyre_default_values{k};
    end
    for k = 1:w
        Rtyre_input_box(k).Value = Rtyre_default_values{k};
    end
    for ii = 1:parameterCount
        cla(hAxes_laptime(ii), 'reset');
    end
    cla(hAxes_tyre_fric(1), 'reset');
    cla(hAxes_LoadTrans(1), 'reset');
    cla(hAxes_LoadTrans(2), 'reset');
    lapTime_label.Text = sprintf('Lap time: ');
end

%% Fetching var values
% All calculations are done in SI units! Need to convert all parameters to SI.
function calculate(~,~, input_box, track_input_box, Ftyre_input_box, Rtyre_input_box, Delta_S, hAxes_laptime, hAxes_tyre_fric, hAxes_LoadTrans, lapTime_label)

    tireF = LoadDependentTireModel();
    tireR = LoadDependentTireModel();
    
    param_input = @(i) str2double(input_box(i).Value);
    
    powertrain_input = @(i) str2num(input_box(i).Value);
  
    vehicle = Vehicle();

    vehicle.mCarTotal               = param_input(1);
    vehicle.rWeightBalF             = param_input(2);
    vehicle.wheelbase               = param_input(3) ./ 1000;
    vehicle.trackWidthF             = param_input(4) ./ 1000;
    vehicle.trackWidthR             = param_input(5) ./ 1000;
    vehicle.hCoG                    = param_input(6) ./ 1000;
    vehicle.hRollCentreF            = param_input(7) ./ 1000;
    vehicle.hRollCentreR            = param_input(8) ./ 1000;
    vehicle.kSpringF                = param_input(9) ./ 1000;
    vehicle.kSpringR                = param_input(10) ./1000;
    vehicle.motionRatioF            = param_input(11);
    vehicle.motionRatioR            = param_input(12);
    vehicle.kWheelARBF              = param_input(13) ./1000;
    vehicle.kWheelARBR              = param_input(14) ./1000;
    vehicle.rAeroBalF               = param_input(15);
    vehicle.aeroDragFactor          = param_input(16);
    vehicle.aeroDownforceFactor     = param_input(17);
    vehicle.rFinalDrive             = param_input(18);
    vehicle.rTransmissionRatio      = param_input(19);
    vehicle.eTransmission           = param_input(20);
    vehicle.nMotorMapLookup         = powertrain_input(21) .* 2 .* pi ./ 60; % rpm to rad/s
    vehicle.MMotorMapLookup         = powertrain_input(22);
    vehicle.tireF                   = tireF;
    vehicle.tireR                   = tireR;

    %% Tyre param
    Ftyre_param_input = @(c) str2double(Ftyre_input_box(c).Value);
    Rtyre_param_input = @(w) str2double(Rtyre_input_box(w).Value);

    % Front Tyre
    tireF.rollingRadius              = Ftyre_param_input(1) ./ 1000;
    tireF.decayCoeff                 = Ftyre_param_input(2);
    tireF.width                      = Ftyre_param_input(3) ./ 1000;
    tireF.muTyreLong_peak            = Ftyre_param_input(4);
    tireF.muTyreLat_peak             = Ftyre_param_input(5);
    tireF.kSpring                    = Ftyre_param_input(6) .* 1000;

    % Rear Tyre
    tireR.rollingRadius             = Rtyre_param_input(1) ./ 1000;
    tireR.decayCoeff                = Rtyre_param_input(2);
    tireR.width                     = Rtyre_param_input(3) ./ 1000;
    tireR.muTyreLong_peak           = Rtyre_param_input(4);
    tireR.muTyreLat_peak            = Rtyre_param_input(5);
    tireR.kSpring                   = Rtyre_param_input(6) .* 1000;
    
    %% Track parameters
    track_input = @(b) str2num(track_input_box(b).Value);

    Radius_corner                   = track_input(1);
    Angle_corner                    = track_input(2);
    Length_straight                 = track_input(3);

    %% Track generation 
    track = SteadyStateTrack(Radius_corner, deg2rad(Angle_corner), Length_straight);

    %% Run the lap
    steadyStateSim = SteadyStateLapSimulation(track, vehicle, distanceStep=Delta_S);
    lap = steadyStateSim.run();

    %% Plot results
    plotResults(lap, hAxes_laptime, hAxes_tyre_fric, hAxes_LoadTrans, lapTime_label);

end

%% Functions

function varargout = plotResults(lap, hAxes_laptime, hAxes_tyre_fric, hAxes_LoadTrans, lapTime_label)
arguments
    lap (1,1) VehicleStates
    hAxes_laptime
    hAxes_tyre_fric
    hAxes_LoadTrans
    lapTime_label
end

PARAMETERS = ["vCar", "gLong", "gLat"];
CONVERSION_FACTORS = [3.6, 1/9.81, 1/9.81];
UNITS = ["kph", "g", "g"];

parameterCount = numel(PARAMETERS);

for ii = 1:parameterCount
    cla(hAxes_laptime(ii), 'reset');
end

for ii = 1:parameterCount 
    hAxes = hAxes_laptime(ii); 
    plotData = lap.results.(PARAMETERS(ii)) .* CONVERSION_FACTORS(ii); 
    rangeY = range(plotData);
    plot(hAxes, lap.results.sRun, plotData, LineWidth=2); 
    grid(hAxes, "on");
    xlabel(hAxes, "sRun (m)");
    ylabel(hAxes, sprintf("%s (%s)", PARAMETERS(ii), UNITS(ii)));
    ylim(hAxes, [min(plotData) - 0.1 * rangeY, max(plotData) + 0.1 * rangeY]);
end

lapTime_label.Text = sprintf('Lap time: %.3fs', lap.results.tRun(end));

if nargout > 1
    varargout{1} = hFig;
end

cla(hAxes_tyre_fric(1), 'reset');
cla(hAxes_LoadTrans(1), 'reset');
cla(hAxes_LoadTrans(2), 'reset');

hAxes2 = hAxes_tyre_fric(1); 
plotDataF = lap.results.muDynamicF;
plotDataR = lap.results.muDynamicR;
plot(hAxes2, lap.results.sRun, plotDataF, 'Color', '#f2b248', 'LineWidth', 1.5, 'DisplayName', 'Front');
hold(hAxes2, 'on');
plot(hAxes2, lap.results.sRun, plotDataR, 'Color', '#7c4081', 'LineWidth', 1.5, 'DisplayName', 'Rear');
hold(hAxes2, 'off');
grid(hAxes2, 'on');
xlabel(hAxes2, 'Distance (m)');
ylabel(hAxes2, 'Friction Coefficient');
title(hAxes2, 'Dynamic Tyre Friction');
legend(hAxes2);

hAxes3 = hAxes_LoadTrans(1);
hAxes4 = hAxes_LoadTrans(2);
plotLLTD = lap.results.LLTD * 100;
plotaRoll = lap.results.aRoll * (180/pi());
plot(hAxes3, lap.results.sRun, plotLLTD, 'Color', '#f2b248', 'LineWidth', 1.5);
hold(hAxes3, 'on');
plot(hAxes4, lap.results.sRun, plotaRoll, 'Color', '#f2b248', 'LineWidth', 1.5);
hold(hAxes4, 'on');
grid(hAxes3, 'on');
grid(hAxes4, 'on');
xlabel(hAxes3, 'Distance (m)');
xlabel(hAxes4, 'Distance (m)');
ylabel(hAxes3, 'LLTD (%)');
ylabel(hAxes4, 'Roll Angle (deg)');

end

% Documentation tab
uitextarea(tab4, ...
    'Value', {
        '';
        ' LAP TIME SIMULATOR';
        ' ================================';
        '';
        ' GUI Version: 4.0 - 23/04/2026';
        '';
        ' HOW TO USE (MAKE SURE IT IS IN FULL SCREEN):';
        '';
        '- Screen is scrollable';
        '- Set vehicle, tyre (both front and rear) and track parameters in the units specified';
        '- If the parameters are a list, ensure that these are separated';
        '  by spaces and spaces ONLY';
        '- Press START to run the simulation';
        '- Press RESET to restore default values';
        '- Doing this will revert ALL values and clear ALL plots, ensure';
        '  that relevant data has been noted before using RESET function';
        '';
        ' OUTPUT TABS';
        '';
        '- Lap Time tab: speed, longitudinal and lateral g in that order';
        '- Tyre Friction tab: dynamic friction front and rear';
        '- Documentation tab: Where you are right now';
        '';
        ' ACKNOWLEDGEMENTS:';
        '';
        '   Aiman Mohd Aminudin';
        '   Victor Pardo Larrosa';
        '   Tom Stevens';
        '   Trin Tancharoen';
        '   Juan Pablo Villalpando Saiz';
        '   Adit Wicaksana';
        '';
        '';
        '';
        '';
        '';
        '';
        '';
        '';
        'If you ain´t first, you´re last (Ricky Bobby)'; 
    }, ...
    'Position', [10, 10, 460, 580], ...
    'Editable', 'off', ...
    'FontSize', 12, ...
    'BackgroundColor', [0.94 0.94 0.94], ...
    'FontColor', 'black');