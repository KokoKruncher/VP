%% ENGR6014: Motorsport Vehicle Performance 2025-2026
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.1

clear; clc; close all;

ParametersVersion = '2025-2026: v1';

% Vehicle parameters:
m                   = 810;    % Vehicle Mass including driver [kg]
D_weight            = 0.45;   % Weight distribution - Front-to-Rear [-] 
L                   = 3020;   % Wheelbase [mm]
trackWidth          = 1500;   % Track width [mm]
h_cog               = 300;    % Centre of gravity height [mm]
LLTD                = 0.5;    % Lateral load transfer distribution
D_aero              = 0.43;   % Aerodynamic balance - Front-to-Rear [-] 
CdA                 = 0.50;   % Aerodynamic drag factor [kg/m]
ClA                 = 0.70;   % Aerodynamic downforce factor [kg/m]
FDR                 = 9.40;   % Final drive ratio [-]
GR                  = 1.00;   % Gear ratio [-]
eta                 = 0.88;   % Final driveline efficiency [-]
Motor_torque_lookup = [0 2000 4000 6000 8000 10000 12000 14000 16000 18000; ...
    360 360 360 360 270 216 180 154 135 120]; % Motor torque lookup

% Tyre model
tyreDecayCoeff      = 0;   % Tyre decay coefficient [-] 3e-5
TyreWidthFront      = 260;
TyreWidthRear       = 380;
R_tyre              = 340;    % Rear tyre rolling radius [mm]
mu_lon              = 1.30;   % Tyre friction coefficient [-], Braking & Acceleration
mu_lat              = 1.36;   % Tyre friction coefficient [-], Cornering

% Circuit properties: 
Radius_corner       = [50 15 35 70];        % Radius of each corner [m]
Angle_corner        = [90 151 38 81];       % Angle of each corner [deg]
Length_straight     = [742 405 368 53];     % Length of each straight [m]

%Simulation parameters
Delta_S             = 0.1;   % Calculation step size interval [m]

%% Parsing inputs
% All calculations are done in SI units! Need to convert all parameters to SI.
track = SteadyStateTrack(Radius_corner, deg2rad(Angle_corner), Length_straight);

tire = LoadDependentTireModel();
tire.radiusTyreRollingRear   = R_tyre ./ 1000;
tire.tyreDecayCoeff          = tyreDecayCoeff;
tire.TyreWidthFront          = TyreWidthFront./1000;
tire.TyreWidthRear           = TyreWidthRear./1000;
tire.muTyreLong_peak         = mu_lon;
tire.muTyreLat_peak          = mu_lat;

vehicle = Vehicle();
vehicle.mCarTotal               = m;
vehicle.rWeightBalF             = D_weight;
vehicle.wheelbase               = L ./  1000;
vehicle.trackWidth              = trackWidth ./ 1000;
vehicle.hCoG                    = h_cog ./ 1000;
vehicle.rMechBalF               = LLTD;
vehicle.rAeroBalF               = D_aero;
vehicle.aeroDragFactor          = CdA;
vehicle.aeroDownforceFactor     = ClA;
vehicle.rFinalDrive             = FDR;
vehicle.rTransmissionRatio      = GR;
vehicle.eTransmission           = eta;
vehicle.nMotorMapLookup         = Motor_torque_lookup(1,:) .* 2 .* pi ./ 60; % rpm to rad/s
vehicle.MMotorMapLookup         = Motor_torque_lookup(2,:);
vehicle.tire = tire;

%% Run the lap
steadyStateSim = SteadyStateLapSimulation(track, vehicle, distanceStep=Delta_S, enableLoggingToCommandWindow=true);
lap = steadyStateSim.run();

%% Print outputs
cornerSpeeds_kph = unique(lap.results.vCar(lap.results.gLong == 0), "stable") .* 3.6;

fprintf("\nLap time = %.3fs\n", lap.results.tRun(end));
fprintf("Corner speeds:\n");
fprintf("%2i: %3.2f kph\n", [(1:numel(cornerSpeeds_kph)).', cornerSpeeds_kph].');

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

%% Plot tire friction
% figure('Name', 'Tire Friction Plot');
% plot(lap.results.sRun, lap.results.muDynamicF, 'Color', '#f2b248', 'LineWidth', 1.5, 'DisplayName', 'Front');
% hold on;
% plot(lap.results.sRun, lap.results.muDynamicR, 'Color', '#7c4081', 'LineWidth', 1.5, 'DisplayName', 'Rear');
% grid on;
% xlabel('Distance (m)')
% ylabel('Friction coefficient');
% title('Dynamic tire friction');
% legend();