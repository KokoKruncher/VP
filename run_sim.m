%% ENGR6014: Motorsport Vehicle Performance 2025-2026
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.0

clear; clc; close all;

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
Motor_torque_lookup = [0 2000 4000 6000 8000 10000 12000 14000 16000 18000; ...
    360 360 360 360 270 216 180 154 135 120]; % Motor torque lookup

% Circuit properties: 
Radius_corner       = [50 15 35 70];        % Radius of each corner [m]
Angle_corner        = [90 151 38 81];       % Angle of each corner [deg]
Length_straight     = [742 405 368 53];     % Length of each straight [m]

%Simulation parameters
Delta_S             = 0.1;   % Calculation step size interval [m]

%% Parsing inputs
% All calculations are done in SI units! Need to convert all parameters to SI.
track = SteadyStateTrack(Radius_corner, deg2rad(Angle_corner), Length_straight);

vehicle = Vehicle();
vehicle.mass = m;
vehicle.weightDistribution = D_weight;
vehicle.wheelbase = L ./  1000;
vehicle.cogHeight = h_cog ./ 1000;
vehicle.rearTyreRollingRadius = R_tyre ./ 1000;
vehicle.muLongitudinal = mu_lon;
vehicle.muLateral = mu_lat;
vehicle.aeroBalance = D_aero;
vehicle.dragFactor = CdA;
vehicle.downforceFactor = ClA;
vehicle.finalDriveRatio = FDR;
vehicle.gearRatio = GR;
vehicle.drivelineEfficiency = eta;
vehicle.motorTorqueLookup_Revs = Motor_torque_lookup(1,:) ./ 60;
vehicle.motorTorqueLookup_Torque = Motor_torque_lookup(2,:);

steadyStateSim = SteadyStateLapSimulation(track, vehicle, distanceStep=Delta_S);
steadyStateSim.run();