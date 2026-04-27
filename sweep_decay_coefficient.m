%% ENGR6014: Motorsport Vehicle Performance 2025-2026
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.1

clear; clc; close all;

decayCoefficients = [0, 1e-5:1e-5:4e-5];

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

sweepCount = numel(decayCoefficients);
tLap = nan(1, sweepCount);
for ii = 1:sweepCount
    tire.tyreDecayCoeff = decayCoefficients(ii);
    vehicle.tire = tire;
    steadyStateSim = SteadyStateLapSimulation(track, vehicle, distanceStep=Delta_S);
    lap = steadyStateSim.run();
    tLap(ii) = lap.results.tRun(end);
end

%% Plot
figure();
plot(decayCoefficients, tLap, "-o", LineWidth=2);
xlabel("Decay coefficient")
ylabel("Lap time (s)")
grid on
