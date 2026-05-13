%% ENGR6014: Motorsport Vehicle Performance 2025-2026
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.1

clear; clc; close all;

steadyStateSim = assembleSim(sweepParameters=["muTyreLong_peak", "muTyreLat_peak"], percentChange=5, isTyreParameter=true);
% steadyStateSim = assembleSim();

percentDeltas = -10:1:10;
Sweeps = struct();
Sweeps.Friction = runSweep(["muTyreLong_peak", "muTyreLat_peak"], percentDeltas, isTyreParameter=true);
Sweeps.Mass = runSweep("mCarTotal", percentDeltas);
Sweeps.WeightDistribution = runSweep("rWeightBalF", percentDeltas);
Sweeps.AeroBalance = runSweep("rAeroBalF", percentDeltas);
Sweeps.AeroLoads = runSweep(["aeroDragFactor", "aeroDownforceFactor"], percentDeltas);
Sweeps.CoGHeight = runSweep("hCoG", percentDeltas);

% lap = steadyStateSim.run();

%% Print outputs
% cornerSpeeds_kph = unique(lap.results.vCar(lap.results.gLong == 0), "stable") .* 3.6;
% 
% fprintf("\nLap time = %.3fs\n", lap.results.tRun(end));
% fprintf("Corner speeds:\n");
% fprintf("%2i: %3.2f kph\n", [(1:numel(cornerSpeeds_kph)).', cornerSpeeds_kph].');

%% Plot results
% plotResults(lap);

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

%% Functions
function Sweep = runSweep(sweepParameters, percentDeltas, args)
arguments
    sweepParameters (1,:) string
    percentDeltas (1,:) double
    args.isTyreParameter (1,1) logical = false
end
sweepCount = numel(percentDeltas);
Sweep = struct();
Sweep.Parameters = sweepParameters;
Sweep.Runs = struct();
for ii = 1:sweepCount
    fprintf(1, "Running sweep %i/%i [%s].\n", ii, sweepCount, join(sweepParameters, ", "));
    steadyStateSim = assembleSim(sweepParameters=sweepParameters, percentChange=percentDeltas(ii), ...
        isTyreParameter=args.isTyreParameter);
    lap = steadyStateSim.run();
    
    Sweep.Runs(ii).PercentDelta = percentDeltas(ii);
    Sweep.Runs(ii).Vehicle = steadyStateSim.vehicle.toStruct();
    Sweep.Runs(ii).Lap = lap.results;
    Sweep.Runs(ii).LapTime = lap.results.tRun(end);
end
end


function steadyStateSim = assembleSim(args)
arguments
    args.sweepParameters (1,:) string = string.empty();
    args.percentChange double {mustBeScalarOrEmpty} = []
    args.isTyreParameter (1,1) logical = false
end
steadyStateSim = assembleBaselineSim();
if isempty(args.sweepParameters)
    return
end

for thisSweepParameter = args.sweepParameters
    changeVehicleParameter(steadyStateSim.vehicle(), thisSweepParameter, args.percentChange, args.isTyreParameter);
end
end


function changeVehicleParameter(vehicle, parameter, percentChange, isTyreParameter)
arguments
    vehicle (1,1) Vehicle
    parameter {mustBeTextScalar}
    percentChange (1,1) double
    isTyreParameter (1,1) logical
end

if ~isTyreParameter 
baselineValue = vehicle.(parameter);
newValue = baselineValue + (baselineValue * percentChange / 100);
vehicle.(parameter) = newValue;
else
    baselineValueF = vehicle.tireF.(parameter);
    newValueF = baselineValueF + (baselineValueF * percentChange / 100);
    vehicle.tireF.(parameter) = newValueF;
    
    baselineValueR = vehicle.tireR.(parameter);
    newValueR = baselineValueR + (baselineValueR * percentChange / 100);
    vehicle.tireR.(parameter) = newValueR;
end
end


function steadyStateSim = assembleBaselineSim()
% ParametersVersion = '2025-2026: v1';

% Vehicle parameters:
m                   = 810;    % Vehicle Mass including driver [kg]
D_weight            = 0.45;   % Weight distribution - Front-to-Rear [-] 
L                   = 3020;   % Wheelbase [mm]
trackWidthF         = 1500;   % Front track width [mm]
trackWidthR         = 1500;   % Rear track width [mm]
h_cog               = 300;    % Centre of gravity height [mm]
D_aero              = 0.43;   % Aerodynamic balance - Front-to-Rear [-]
CdA                 = 0.50;   % Aerodynamic drag factor [kg/m]
ClA                 = 0.70;   % Aerodynamic downforce factor [kg/m]
FDR                 = 9.40;   % Final drive ratio [-]
GR                  = 1.00;   % Gear ratio [-]
eta                 = 0.88;   % Final driveline efficiency [-]
Motor_torque_lookup = [0 2000 4000 6000 8000 10000 12000 14000 16000 18000; ...
    360 360 360 360 270 216 180 154 135 120]; % Motor torque lookup
hRollCentreF        = 8;      % Front roll centre height [mm]
hRollCentreR        = 40;     % Rear roll centre height [mm]
kSpringF            = 175;    % Front corner spring rate [N/mm]
kSpringR            = 150;    % Rear corner spring rate [N/mm]
motionRatioF        = 1.18;   % Front motion ratio (wheel/spring) [-]
motionRatioR        = 1.15;   % Rear motion ratio (wheel/spring) [-]
kWheelARBF          = 260;    % Front ARB spring rate equivalent at wheel [N/mm]
kWheelARBR          = 30;     % Rear ARB spring rate equivalent at wheel [N/mm}

% Tyre model
tyreDecayCoeff      = 0;   % Tyre decay coefficient [-] 3e-5
TyreWidthFront      = 260;
TyreWidthRear       = 380;
R_tyre              = 340;    % Rear tyre rolling radius [mm]
mu_lon              = 1.30;   % Tyre friction coefficient [-], Braking & Acceleration
mu_lat              = 1.36;   % Tyre friction coefficient [-], Cornering
kTyreF              = 210;    % Front tyre spring rate [N/mm]
kTyreR              = 245;    % Rear tyre spring rate [N/mm]

% Circuit properties: 
Radius_corner       = [50 15 35 70];        % Radius of each corner [m]
Angle_corner        = [90 151 38 81];       % Angle of each corner [deg]
Length_straight     = [742 405 368 53];     % Length of each straight [m]

%Simulation parameters
Delta_S             = 0.1;   % Calculation step size interval [m]

% Parsing inputs
% All calculations are done in SI units! Need to convert all parameters to SI.
track = SteadyStateTrack(Radius_corner, deg2rad(Angle_corner), Length_straight);

tireF = LoadDependentTireModel();
tireF.rollingRadius           = R_tyre ./ 1000;
tireF.decayCoeff              = tyreDecayCoeff;
tireF.width                   = TyreWidthFront./1000;
tireF.muTyreLong_peak         = mu_lon;
tireF.muTyreLat_peak          = mu_lat;
tireF.kSpring                 = kTyreF .* 1000;

tireR                         = LoadDependentTireModel();
tireR.rollingRadius            = R_tyre ./ 1000;
tireR.decayCoeff              = tyreDecayCoeff;
tireR.width                   = TyreWidthRear./1000;
tireR.muTyreLong_peak         = mu_lon;
tireR.muTyreLat_peak          = mu_lat;
tireR.kSpring                 = kTyreR .* 1000;

vehicle = Vehicle();
vehicle.mCarTotal               = m;
vehicle.rWeightBalF             = D_weight;
vehicle.wheelbase               = L ./  1000;
vehicle.trackWidthF             = trackWidthF ./ 1000;
vehicle.trackWidthR             = trackWidthR ./ 1000;
vehicle.hCoG                    = h_cog ./ 1000;
vehicle.hRollCentreF            = hRollCentreF ./ 1000;
vehicle.hRollCentreR            = hRollCentreR ./ 1000;
vehicle.kSpringF                = kSpringF .* 1000;
vehicle.kSpringR                = kSpringR .* 1000;
vehicle.motionRatioF            = motionRatioF;
vehicle.motionRatioR            = motionRatioR;
vehicle.kWheelARBF              = kWheelARBF .* 1000;
vehicle.kWheelARBR              = kWheelARBR .* 1000;
vehicle.rAeroBalF               = D_aero;
vehicle.aeroDragFactor          = CdA;
vehicle.aeroDownforceFactor     = ClA;
vehicle.rFinalDrive             = FDR;
vehicle.rTransmissionRatio      = GR;
vehicle.eTransmission           = eta;
vehicle.nMotorMapLookup         = Motor_torque_lookup(1,:) .* 2 .* pi ./ 60; % rpm to rad/s
vehicle.MMotorMapLookup         = Motor_torque_lookup(2,:);
vehicle.tireF                   = tireF;
vehicle.tireR                   = tireR;

steadyStateSim = SteadyStateLapSimulation(track, vehicle, distanceStep=Delta_S, enableLoggingToCommandWindow=false);
end


function varargout = plotResults(lap)
arguments
    lap (1,1) VehicleStates
end
PARAMETERS = ["vCar", "gLong", "gLat", "LLTD", "aRoll"];
CONVERSION_FACTORS = [3.6, 1/9.81, 1/9.81, 100, 180./pi];
UNITS = ["kph", "g", "g", "%", "deg"];

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