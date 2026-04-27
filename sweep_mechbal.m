%% ENGR6014: Motorsport Vehicle Performance 2025-2026
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.1

clear; clc; close all;

% List out the ARB rates to try
kWheelARBFSweep = linspace(0, 300, 7);
kWheelARBRSweep = linspace(0, 600, 7);
[kWheelARBFSweep, kWheelARBRSweep] = meshgrid(kWheelARBFSweep, kWheelARBRSweep);
LLTDSweep = nan(size(kWheelARBFSweep));

%% Select track
% Circuit properties: Default test track
% Radius_corner       = [50 15 35 70];        % Radius of each corner [m]
% Angle_corner        = [90 151 38 81];       % Angle of each corner [deg]
% Length_straight     = [742 405 368 53];     % Length of each straight [m]



% Circuit properties: Sao Paulo ePrix (11 Corners, 12 Straights)
% Radius_corner   = [18, 18, 12, 25, 22, 25, 14, 20, 60, 15, 18];       
% Angle_corner    = [90, 90, 180, 45, 90, 45, 140, 80, 25, 90, 90];     
% Length_straight = [650, 80, 80, 600, 80, 80, 500, 120, 350, 80, 80];



% Formula E - Berlin Tempelhof Circuit Profile
% 1: Hairpin Left, 2-3: Fast kinks, 4: Left 90, 5: Right 90, 
% 6: Tight Left, 7-8: Kinks, 9: Hairpin Right, 10: Sweeping Right
% Radius of each corner [m]
% Radius_corner = [15,60,55,20,25,15,80,80,12,35];
% 
% % Angle of each corner [deg] (Deflection angle)
% Angle_corner = [170,30,45,90,90,140,20,20,180,90];
% 
% % Length of straight leading into the corner [m]
% Length_straight = [450,50,50,150,80,120,400,40,150,100];



% % Silverstone GP Circuit - Complete Track Profile
% % 1: Abbey, 2: Farm, 3: Village, 4: Loop, 5: Aintree, 6: Brooklands, 7: Luffield, 
% % 8: Woodcote, 9: Copse, 10-14: Maggotts/Becketts/Chapel, 15: Stowe, 16: Vale, 17-18: Club
% 
% % Radius of each corner [m]
% Radius_corner = [120,160,35,18,70,65,45,180,110,140,90,60,45,100,105,30,55,130];
% 
% % Angle of each corner [deg]
% Angle_corner = [45,30,100,160,40,105,195,25,80,35,45,75,95,30,115,100,60,40];
% 
% % Length of straight leading into the corner [m]
% Length_straight = [296,120,150,60,210,550,80,150,420,450,50,40,35,60,770,420,40,120];

%% Baseline parameters
ParametersVersion = '2025-2026: v1';

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
tyreDecayCoeff      = 3e-5;   % Tyre decay coefficient [-] 3e-5
TyreWidthFront      = 260;
TyreWidthRear       = 380;
R_tyre              = 340;    % Rear tyre rolling radius [mm]
mu_lon              = 1.30;   % Tyre friction coefficient [-], Braking & Acceleration
mu_lat              = 1.36;   % Tyre friction coefficient [-], Cornering
kTyreF              = 210;    % Front tyre spring rate [N/mm]
kTyreR              = 245;    % Rear tyre spring rate [N/mm]

%Simulation parameters
Delta_S             = 0.1;   % Calculation step size interval [m]

%% Parsing inputs
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

%% Run the lap
sweepCount = numel(kWheelARBFSweep);
tLap = nan(size(LLTDSweep));
for ii = 1:sweepCount
    fprintf("Running lap %i/%i\n", ii, sweepCount);
    vehicle.kWheelARBF = kWheelARBFSweep(ii) .* 1000;
    vehicle.kWheelARBR = kWheelARBRSweep(ii) .* 1000;
    steadyStateSim = SteadyStateLapSimulation(track, vehicle, distanceStep=Delta_S);
    lap = steadyStateSim.run();
    tLap(ii) = lap.results.tRun(end);
    LLTDSweep(ii) = mean(lap.results.LLTD(isfinite(lap.results.LLTD)));
end

%% Fit
% fitObj = polyfit(LLTDSweep(:) * 100, tLap(:), 1);
% sensitivity_ms_per_percent = fitObj(1) * 1000;
% fprintf("\nLLTD sensitivity (ms/%%) = %.2f\n", sensitivity_ms_per_percent);

%% Plot
figure();
surf(kWheelARBFSweep, kWheelARBRSweep, tLap);
xlabel("Front ARB wheel rate (N/mm)")
ylabel("Rear ARB wheel rate (N/mm)")
zlabel("Lap time (s)")
grid on

figure();
scatter(LLTDSweep(:) * 100, tLap(:));
xlabel("LLTD (%)")
ylabel("Lap time (s)");

