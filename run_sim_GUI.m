%% ENGR6014: Motorsport Vehicle Performance 2025-2026
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.0

clear; clc; close all;

figure('Name', 'Lap Time Simulator', 'NumberTitle', 'off', 'Position', [100, 50, 1200, 700], 'Color', [0.94 0.94 0.94], 'Scrollable', 'on');

% Column for the vehicle Parameters

uicontrol('Style', 'text', 'String', 'Vehicle Parameters', 'Position', [10 780 200 25], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'ForegroundColor', 'black');

vehicle_data = {'Vehicle Mass', 'Weight Dist Front', 'Wheelbase', 'CG Height', 'Rear Tyre Radius', 'Longitudinal tyre Mu', 'Lateral tyre Mu', 'Aero Balance front', 'Aero Drag Factor', 'Aero Downforce factor', 'Final drive ratio', 'Gear Ratio', 'Final driveline efficiency'};

default_values = {'810', '0.45', '3020', '300', '340', '1.30', '1.36', '0.43', '0.50', '0.70', '9.40', '1.00', '0.88'};

value_units = {'kg', 'fraction', 'mm', 'mm', 'mm', 'Value', 'Value', 'Fraction', 'Kg/m', 'Kg/m', 'Ratio', 'Ratio', 'Fraction'};

n = length(vehicle_data);
input_box = gobjects(n,1);

for i = 1:n
    y = 750 - (i-1)*28;
    uicontrol('Style', 'text', 'String', vehicle_data{i}, 'Position', [10,y,175,20], 'FontSize', 8, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'ForegroundColor', 'black');
    input_box(i) = uicontrol('Style', 'edit', 'String', default_values{i}, 'Position', [190,y+5,75,20], 'FontSize', 8, 'BackgroundColor', 'white', 'ForegroundColor', 'black');
    uicontrol('Style', 'text', 'String', value_units{i}, 'Position', [270,y,60,20], 'FontSize', 8, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'ForegroundColor', 'black');
end

% Control buttons

uicontrol('Style', 'pushbutton', 'String', 'START', 'Position', [570,560,120,35], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.2 0.6 0.2], 'ForegroundColor', 'white', 'Callback', @calculate);

uicontrol('Style', 'pushbutton', 'String', 'RESET', 'Position', [570,520,120,35], 'FontSize', 10, 'BackgroundColor', [0.7 0.2 0.2], 'ForegroundColor', 'white', 'Callback', @reset);

function reset(~,~)
    for k = 1:n
        set(input_box(k),'String',default_values{k});
    end
end

%{
ParametersVersion = '2025-2026: v1';

% Vehicle parameters:
m                   = 810;    % Vehicle Mass including driver [kg]
D_weight            = 0.45;   % Weight distribution - Front-to-Rear [-] 
L                   = 3020;   % Wheelbase [mm]
h_cog               = 300;    % Centre of gravity height [mm]
R_tyre              = 340;    % Rear tyre rolling radius [mm]
mu_lon              = 1.30;   % Tyre friction coefficient [-], Braking & Acceleration
mu_lat              = 1.36;   % Tyre friction coefficient [-], Cornering
D_aero              = 0.43;   % Aerodynamic balance - Front-to-Rear [-] 
CdA                 = 0.50;   % Aerodynamic drag factor [kg/m]
ClA                 = 0.70;   % Aerodynamic downforce factor [kg/m]
FDR                 = 9.40;   % Final drive ratio [-]
GR                  = 1.00;   % Gear ratio [-]
eta                 = 0.88;   % Final driveline efficiency [-]
%}
 % Motor torque lookup
 %% Moved to calculate


% Circuit parameters: 

uicontrol('Style', 'text', 'String', 'Track Parameters', 'Position', [300 780 200 25], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.94 0.94 0.94], 'ForegroundColor', 'black');

track_data = {'Corner Radius', 'Corner Angle', 'Length Straight'};

track_default_values = {'50 15 35 70', '90 151 38 81', '742 405 368 53'};

track_units = {'m', 'Deg', 'm'};

l = length(track_data);

for b = 1:l
    y = 750 - (b-1)*28;
    uicontrol('Style', 'text', 'String', track_data{b}, 'Position', [350 y 200 25], 'FontSize', 8, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'ForegroundColor', 'black');
    input_box(b) = uicontrol('Style', 'edit', 'String', track_default_values{b}, 'Position', [530,y+5,125,20], 'FontSize', 8, 'BackgroundColor', 'white', 'ForegroundColor', 'black');
    uicontrol('Style', 'text', 'String', track_units{b}, 'Position', [660,y,30,20], 'FontSize', 8, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], 'ForegroundColor', 'black');
end

%{
Radius_corner       = [50 15 35 70];        % Radius of each corner [m]
Angle_corner        = [90 151 38 81];       % Angle of each corner [deg]
Length_straight     = [742 405 368 53];     % Length of each straight [m]
%}

%Simulation parameters:
Delta_S             = 0.1;   % Calculation step size interval [m]

%% Parsing inputs
% All calculations are done in SI units! Need to convert all parameters to SI.
function calculate(~,~)
    
    Motor_torque_lookup = [0 2000 4000 6000 8000 10000 12000 14000 16000 18000; ...
    360 360 360 360 270 216 180 154 135 120];

    track = SteadyStateTrack(Radius_corner, deg2rad(Angle_corner), Length_straight);
    param_input = @(i) str2double(get(input_box(i),'String'));

    vehicle = Vehicle();
    vehicle.mCarTotal               = param_input(1);
    vehicle.rWeightBalF             = param_input(2);
    vehicle.wheelbase               = param_input(3) ./ 1000;
    vehicle.hCoG                    = param_input(4) ./ 1000;
    vehicle.radiusTyreRollingRear   = param_input(5) ./ 1000;
    vehicle.muTyreLong              = param_input(6);
    vehicle.muTyreLat               = param_input(7);
    vehicle.rAeroBalF               = param_input(8);
    vehicle.aeroDragFactor          = param_input(9);
    vehicle.aeroDownforceFactor     = param_input(10);
    vehicle.rFinalDrive             = param_input(11);
    vehicle.rTransmissionRatio      = param_input(12);
    vehicle.eTransmission           = param_input(13);
    vehicle.nMotorMapLookup         = Motor_torque_lookup(1,:) .* 2 .* pi ./ 60; % rpm to rad/s
    vehicle.MMotorMapLookup         = Motor_torque_lookup(2,:);

    %% Track parameters
    track_input = @(b) str2num(get(input_box(b),'String'));

    Radius_corner                   = track_input(1);
    Angle_corner                    = track_input(2);
    Length_straight                 = track_input(3);
end
%% Run the lap
steadyStateSim = SteadyStateLapSimulation(track, vehicle, distanceStep=Delta_S);
lap = steadyStateSim.run();

%% Plot results
plotResults(lap);

%% Functions

function varargout = plotResults(lap)
arguments
    lap (1,1) VehicleStates
end
PARAMETERS = ["vCar", "gLong", "gLat"];
CONVERSION_FACTORS = [3.6, 1/9.81, 1/9.81];
UNITS = ["kph", "g", "g"];

parameterCount = numel(PARAMETERS);
hFig = figure(Visible="off");
hTiles = tiledlayout(hFig, parameterCount, 1);
for ii = 1:parameterCount
    hAxes = nexttile(hTiles);
    y = lap.results.(PARAMETERS(ii)) .* CONVERSION_FACTORS(ii);
    rangeY = range(y);
    plot(hAxes, lap.results.sRun, y, LineWidth=2);
    grid(hAxes, "on");
    xlabel(hAxes, "sRun (m)");
    ylabel(hAxes, sprintf("%s (%s)", PARAMETERS(ii), UNITS(ii)));
    ylim(hAxes, [min(y) - 0.1 * rangeY, max(y) + 0.1 * rangeY]);
end
title(hTiles, sprintf("Lap time = %.3fs", lap.results.tRun(end)));
hFig.Visible = "on";

if nargout > 1
    varargout{1} = hFig;
end
end