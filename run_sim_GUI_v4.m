%% ENGR6014: Motorsport Vehicle Performance 2025-2026
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.0

clear; clc; close all;

% Find the middle of the screen

screensize = get(0, 'ScreenSize');
WindowWidth = 900;
WindowHeight = 800;
leftpos = (screensize(3) - WindowWidth) / 2;
bottompos = (screensize(4) - WindowHeight) / 2;

Window = uifigure('Name', 'Lap Time Simulator', 'NumberTitle', 'off', 'Position', [leftpos, bottompos, WindowWidth, WindowHeight], 'Color', [0.94 0.94 0.94], 'Scrollable', 'off');

% Column for the vehicle Parameters

uilabel(Window, 'text', 'Vehicle Parameters', 'Position', [10 780 200 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

vehicle_data = {'Vehicle Mass', 'Weight Dist Front', 'Wheelbase', 'CG Height', 'Aero Balance front', 'Aero Drag Factor', 'Aero Downforce factor', 'Final drive ratio', 'Gear Ratio', 'Final driveline efficiency', 'Motor RPM', 'Motor Torque'};

default_values = {'810', '0.45', '3020', '300', '0.43', '0.50', '0.70', '9.40', '1.00', '0.88', '0 2000 4000 6000 8000 10000 12000 14000 16000 18000', '360 360 360 360 270 216 180 154 135 120'};

value_units = {'kg', 'fraction', 'mm', 'mm', 'Fraction', 'Kg/m', 'Kg/m', 'Ratio', 'Ratio', 'Fraction', 'RPM', 'Nm'};

n = length(vehicle_data);
input_box = gobjects(n, 1);
track_input_box = gobjects(n, 1);
tyre_input_box = gobjects(n, 1);

for i = 1:n
    y = 750 - (i-1)*28;
    uilabel(Window, 'text', vehicle_data{i}, 'Position', [10,y,175,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
    input_box(i) = uieditfield(Window, 'text', 'Value', default_values{i}, 'Editable', 'on', 'Position', [190,y+5,75,20], 'FontSize', 12, 'BackgroundColor', 'white', 'FontColor', 'black');
    uilabel(Window, 'text',  value_units{i}, 'Position', [270,y,60,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
end

% Circuit parameters: 

uilabel(Window, 'text', 'Track Parameters', 'Position', [350 780 200 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

track_data = {'Corner Radius', 'Corner Angle', 'Length Straight'};

track_default_values = {'50 15 35 70', '90 151 38 81', '742 405 368 53'};

track_units = {'m', 'Deg', 'm'};

l = length(track_data);

for b = 1:l
    y = 750 - (b-1)*28;
    uilabel(Window, 'text', track_data{b}, 'Position', [350 y 200 25], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
    track_input_box(b) = uieditfield(Window, 'Text', 'Value', track_default_values{b}, 'Position', [530,y+5,125,20], 'FontSize', 12, 'BackgroundColor', 'white', 'FontColor', 'black');
    uilabel(Window, 'text', track_units{b}, 'Position', [660,y,30,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
end

% Tyre Parameters:

y0 = 750 - (i-1)*28;
yd = y0 - 35;

uilabel(Window, 'text', 'Tyre Parameters', 'Position', [10 yd 200 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

tyre_data = {'Rear Rolling Radius', 'Decay Coefficient', 'Front Width', 'Rear Width', 'Longitudinal peak Mu', 'Lateral Peak Mu'};

tyre_default_values = {'340', '3e-5', '260', '380', '1.30', '1.36'};

tyre_units = {'mm', 'Value', 'mm', 'mm', 'Value', 'Value'};

s = length(tyre_data);

for c = 1:s
    y = (yd-30) - (c - 1)* 28;
    uilabel(Window, 'text', tyre_data{c}, 'Position', [10 y 200 25], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
    tyre_input_box(c) = uieditfield(Window, 'Text', 'Value', tyre_default_values{c}, 'Position', [190,y+5,75,20], 'FontSize', 12, 'BackgroundColor', 'white', 'FontColor', 'black');
    uilabel(Window, 'text', tyre_units{c}, 'Position', [270,y,30,20], 'FontSize', 12, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');
end

%Simulation parameters:
Delta_S             = 0.1;   % Calculation step size interval [m]

% Axes placeholder in Tab visualisation
axHeight = 180; 
axWidth = 460; 
axLeft = 360;

parameterCount = 3;
parameterCount2 = 1;

tabVis = uitabgroup(Window, 'Position', [350 30 490 650]);

tab1 = uitab(tabVis, 'Title', 'Lap Time');
tab2 = uitab(tabVis, 'Title', 'Tyre Friction');
tab3 = uitab(tabVis, 'Title', 'Documentation');

lapTime_label = uilabel(tab1, 'text', 'Lap time: ', 'Position', [100 590 250 25], 'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'FontColor', 'black');

hAxes_laptime = gobjects(parameterCount, 1);
hAxes_tyre_fric = gobjects(parameterCount2, 1);

for ii = 1:parameterCount 
    axBottom = 10 + (parameterCount - ii) * (axHeight + 20); 
    hAxes_laptime(ii) = uiaxes(tab1, 'Position', [10, axBottom, axWidth, axHeight]); 
end

hAxes_tyre_fric(1) = uiaxes(tab2, 'Position', [10, 300, axWidth, 300]);

% Control buttons

uibutton(Window, 'Text', 'START', 'Position', [720,740,120,35], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'white', 'ButtonPushedFcn', @(src,evt) calculate(src, evt, input_box, track_input_box, tyre_input_box, Delta_S, hAxes_laptime, hAxes_tyre_fric, lapTime_label));

uibutton(Window, 'Text', 'RESET', 'Position', [720,700,120,35], 'FontSize', 10, 'BackgroundColor', [0.7 0.2 0.2], 'FontColor', 'white', 'ButtonPushedFcn', @(src,evt) reset(src, evt, n, l, s, parameterCount, input_box, track_input_box, tyre_input_box, default_values, track_default_values, tyre_default_values, hAxes_laptime, hAxes_tyre_fric, lapTime_label));

function reset(~,~, n, l, s, parameterCount, input_box, track_input_box, tyre_input_box, default_values, track_default_values, tyre_default_values, hAxes_laptime, hAxes_tyre_fric, lapTime_label)
    for k = 1:n
        input_box(k).Value = default_values{k};
    end
    for k = 1:l
        track_input_box(k).Value = track_default_values{k};
    end
    for k = 1:s
        tyre_input_box(k).Value = tyre_default_values{k};
    end
    for ii = 1:parameterCount
        cla(hAxes_laptime(ii), 'reset');
    end
    cla(hAxes_tyre_fric(1), 'reset');
    lapTime_label.Text = sprintf('Lap time: ');
end

%% Fetching var values
% All calculations are done in SI units! Need to convert all parameters to SI.
function calculate(~,~, input_box, track_input_box, tyre_input_box, Delta_S, hAxes_laptime, hAxes_tyre_fric, lapTime_label)

    tire = LoadDependentTireModel();

    param_input = @(i) str2double(input_box(i).Value);
    
    powertrain_input = @(i) str2num(input_box(i).Value);
  
    vehicle = Vehicle();

    vehicle.mCarTotal               = param_input(1);
    vehicle.rWeightBalF             = param_input(2);
    vehicle.wheelbase               = param_input(3) ./ 1000;
    vehicle.hCoG                    = param_input(4) ./ 1000;
    vehicle.rAeroBalF               = param_input(5);
    vehicle.aeroDragFactor          = param_input(6);
    vehicle.aeroDownforceFactor     = param_input(7);
    vehicle.rFinalDrive             = param_input(8);
    vehicle.rTransmissionRatio      = param_input(9);
    vehicle.eTransmission           = param_input(10);
    vehicle.nMotorMapLookup         = powertrain_input(11) .* 2 .* pi ./ 60; % rpm to rad/s
    vehicle.MMotorMapLookup         = powertrain_input(12);
    vehicle.tire = tire;

    %% Tyre param
    tyre_param_input = @(c) str2double(tyre_input_box(c).Value);

    tire.radiusTyreRollingRear      = tyre_param_input(1) ./ 1000;
    tire.tyreDecayCoeff             = tyre_param_input(2);
    tire.TyreWidthFront             = tyre_param_input(3) ./1000;
    tire.TyreWidthRear              = tyre_param_input(4) ./1000;
    tire.muTyreLong_peak            = tyre_param_input(5);
    tire.muTyreLat_peak             = tyre_param_input(6);

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
    plotResults(lap, hAxes_laptime, hAxes_tyre_fric, lapTime_label);

end

%% Functions

function varargout = plotResults(lap, hAxes_laptime, hAxes_tyre_fric, lapTime_label)
arguments
    lap (1,1) VehicleStates
    hAxes_laptime
    hAxes_tyre_fric
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

end

% Documentation tab
uitextarea(tab3, ...
    'Value', {
        '';
        ' LAP TIME SIMULATOR';
        ' ================================';
        '';
        ' GUI Version: 4.0 - 23/04/2026';
        '';
        ' HOW TO USE:';
        '';
        '- Set vehicle, tyre and track parameters in the units specified';
        '- If the parameters are a list, ensure that these are separated';
        '  by spaces and spaces ONLY';
        '- Press START to run the simulation';
        '- Press RESET to restore default values';
        '- Doing this will revert ALL values and clear ALL plots, ensure';
        '  that relevant data has been noted before using RESET function';
        '';
        ' OUTPUT TABS';
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
        '   Juan Pablo Villalpando Saiz'
        '   Adit Wicaksana'
    }, ...
    'Position', [10, 10, 460, 580], ...
    'Editable', 'off', ...
    'FontSize', 12, ...
    'BackgroundColor', [0.94 0.94 0.94], ...
    'FontColor', 'black');