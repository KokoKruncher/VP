% Requires ColorBrewer package from FileExchange:
% https://uk.mathworks.com/matlabcentral/fileexchange/45208-colorbrewer-attractive-and-distinctive-colormaps

clear; clc; close all;

load("Outputs\CW3_Sweeps.mat", "Sweeps");

%% Calculate Sensitivity
sweepNames = convertCharsToStrings(fieldnames(Sweeps));
sweepCount = numel(sweepNames);
for ii = 1:sweepCount
    thisSweepName = sweepNames(ii);
    percentDelta = [Sweeps.(thisSweepName).Runs.PercentDelta];
    laptime = [Sweeps.(thisSweepName).Runs.LapTime];
    baselineLaptime = laptime(percentDelta == 0);

    assert(isscalar(baselineLaptime), "Need to have 1 baseline laptime.");
    laptimeDelta = laptime - baselineLaptime;
    linearFitCoefficients = polyfit(percentDelta, laptimeDelta, 1);

    Sweeps.(thisSweepName).Results.PercentDelta = percentDelta;
    Sweeps.(thisSweepName).Results.LaptimeDelta = laptimeDelta;
    Sweeps.(thisSweepName).Results.Sensitivity = linearFitCoefficients(1);
end

%% Plot
sweepNameToPrettyName = dictionary();
sweepNameToPrettyName("Friction") = "Friction";
sweepNameToPrettyName("Mass") = "Mass";
sweepNameToPrettyName("WeightDistribution") = "Weight Distribution";
sweepNameToPrettyName("AeroBalance") = "Aero Balance";
sweepNameToPrettyName("AeroLoads") = "Drag & Downforce";
sweepNameToPrettyName("CoGHeight") = "CoG Height";

% plotAllSweeps(Sweeps);
plotAllAbsoluteSensitivities(Sweeps, sweepNameToPrettyName);
hFigIndividual = plotIndividualSweep(Sweeps, sweepNameToPrettyName);

%% Functions
function vCarCorner = extractCornerSpeeds(lap)
arguments
    lap table
end
isCorner = ~isnan(lap.rCorner);
vCarCorner = lap.vCar(isCorner);
vCarCorner = unique(vCarCorner, "stable");
end


function AccelProfile = getAccelProfile(lap)
arguments
    lap table
end
% Shift gLat forwards by 5 samples to get rid of the first few parts of the straight where load transfer hasn't fully
% settled yet.
isAccel = lap.gLong > 0 & circshift(lap.gLat, 5) == 0;
gLong = lap.gLong(isAccel);
vCar = lap.vCar(isAccel);

% Sort to make line plots work
[vCar, sortOrder] = sort(vCar, "ascend");
gLong = gLong(sortOrder);

AccelProfile = struct();
AccelProfile.vCar = vCar;
AccelProfile.gLong = gLong;
end


function BrakingProfile = getBrakingProfile(lap)
arguments
    lap table
end
isBraking = lap.gLong < 0;
gLong = lap.gLong(isBraking);
vCar = lap.vCar(isBraking);

% Sort to make line plots work
[vCar, sortOrder] = sort(vCar, "ascend");
gLong = gLong(sortOrder);

BrakingProfile = struct();
BrakingProfile.vCar = vCar;
BrakingProfile.gLong = gLong;
end


function BaselineParameters = getBaselineParameters(Sweep)
arguments
    Sweep (1,1) struct
end
isBaseline = [Sweep.Runs.PercentDelta] == 0;
assert(nnz(isBaseline) == 1);
baselineVehicle = Sweep.Runs(isBaseline).Vehicle;
parameterNames = Sweep.Parameters;
parameterCount = numel(parameterNames);
BaselineParameters = struct();
for ii = 1:parameterCount
    thisParameterName = parameterNames(ii);
    if isfield(baselineVehicle, thisParameterName)
        baselineParameter = baselineVehicle.(thisParameterName);
    else
        baselineParameter = baselineVehicle.tireF.(thisParameterName);
    end
    BaselineParameters.(thisParameterName) = baselineParameter;
end
end


function plotAllSweeps(Sweeps)
sweepNames = convertCharsToStrings(fieldnames(Sweeps));
sweepCount = numel(sweepNames);

hFig = figure(Name="All Sweeps", NumberTitle="off");
hAxes = axes(hFig);
hold(hAxes, "on");
for ii = 1:sweepCount
    thisSweepName = sweepNames(ii);
    percentDelta = [Sweeps.(thisSweepName).Runs.PercentDelta];
    laptime = [Sweeps.(thisSweepName).Runs.LapTime];
    baselineLaptime = laptime(percentDelta == 0);
    laptimeDelta = laptime - baselineLaptime;
    plot(hAxes, percentDelta, laptimeDelta, "-o", DisplayName=thisSweepName);
end
hold(hAxes, "off");
grid(hAxes, "on");
legend(hAxes);
xlabel(hAxes, "Parameter delta to baseline (%)");
ylabel(hAxes, "Lap time delta (s)");
end


function plotAllAbsoluteSensitivities(Sweeps, sweepNameToPrettyName)
sweepNames = convertCharsToStrings(fieldnames(Sweeps));
sweepCount = numel(sweepNames);
sensitivities = nan(sweepCount, 1);
for ii = 1:sweepCount
    sensitivities(ii) = Sweeps.(sweepNames(ii)).Results.Sensitivity;
end
[sensitivities, sortOrder] = sort(abs(sensitivities), "descend");
sweepNames = sweepNames(sortOrder);
hFig = figure(Name="All Sensitivities", NumberTitle="off");
hAxes = axes(hFig);
hBar = bar(hAxes, sweepNameToPrettyName(sweepNames), abs(sensitivities) * 1000);
hBar.Labels = round(hBar.YData, 1);
ylabel(hAxes, "Sensitivity (ms/%)")
title(hAxes, "Absolute laptime sensitivity (non-directional)")
box(hAxes, "off");
hAxes.XAxis.TickLength = [0, 0];
fontsize(hFig, 16, "points");
hFig.Units = "normalized";
hFig.OuterPosition = [0.25 0.25 0.5 0.5];
end


function hFig = plotIndividualSweep(Sweeps, sweepNameToPrettyName)
arguments
    Sweeps (1,1) struct
    sweepNameToPrettyName dictionary
end
sweepNames = convertCharsToStrings(fieldnames(Sweeps));
iFig = 0;
for thisSweepName = sweepNames(:).'
    
    prettyName = sweepNameToPrettyName(thisSweepName);
    ThisSweep = Sweeps.(thisSweepName);
    sensitivity_ms_per_percent = ThisSweep.Results.Sensitivity * 1000;

    % Laptime delta
    iFig = iFig + 1;
    hFig(iFig) = figure(Name=sprintf("%s Sweep - Laptime delta", thisSweepName), NumberTitle="off");
    hAxes = axes(hFig(iFig));
    plot(hAxes, ThisSweep.Results.PercentDelta, ThisSweep.Results.LaptimeDelta, "-o");
    xlabel(hAxes, "Change from baseline (%)");
    ylabel(hAxes, "Laptime delta (s)");
    grid(hAxes, "on");
    title(hAxes, "Laptime deltas from baseline");
    box(hAxes, "off");
    sensitivityString = sprintf("%s Sensitivity: %.0f ms/%%", prettyName, sensitivity_ms_per_percent);
    BaselineParameters = getBaselineParameters(ThisSweep);
    baselineParameterNames = convertCharsToStrings(fieldnames(BaselineParameters));
    baselineParameterCount = numel(baselineParameterNames);
    baselineParameterStrings = string.empty();
    for iParameter = baselineParameterCount:-1:1
        thisParameterName = baselineParameterNames(iParameter);
        thisParameterValue = BaselineParameters.(thisParameterName);
        baselineParameterStrings(iParameter,1) = sprintf("Baseline %s: %.2f", thisParameterName, thisParameterValue);
    end
    titleString = join([sensitivityString; baselineParameterStrings], newline);
    hTitle = title(hAxes, titleString);
    hTitle.Interpreter = "none";

    % Corner speeds
    runCount = numel(ThisSweep.Runs);
    runIndices = downsample(1:runCount, 4);
    plottedRunCount = numel(runIndices);
    percentDelta = [ThisSweep.Runs(runIndices).PercentDelta];
    vCarCornerBaseline = extractCornerSpeeds(ThisSweep.Runs([ThisSweep.Runs.PercentDelta] == 0).Lap);
    vCarCornerBaseline = reshape(vCarCornerBaseline, 1, []);
    vCarCorner = cell(plottedRunCount, 1);
    for iRun = runIndices(:).'
        lap = ThisSweep.Runs(iRun).Lap;
        vCarCorner{iRun} = reshape(extractCornerSpeeds(lap), 1, []);
    end
    vCarCorner = vertcat(vCarCorner{:});
    vCarCornerDelta = vCarCorner - vCarCornerBaseline;
    cornerCount = width(vCarCorner);
    iFig = iFig + 1;
    hFig(iFig) = figure(Name=sprintf("%s Sweep - Corner speed deltas", thisSweepName), NumberTitle="off");
    hAxes = axes(hFig(iFig));
    hBar = bar("T" + (1:cornerCount), vCarCornerDelta .* 3.6, FaceColor="flat");
    colourMap = brewermap(plottedRunCount, "Reds");
    for iBar = 1:plottedRunCount
        hBar(iBar).CData = repmat(colourMap(iBar,:), cornerCount, 1);
    end
    xlabel(hAxes, "Corner")
    ylabel(hAxes, "Corner speed delta from baseline (kph)");
    legendLabels = arrayfun(@(x) sprintf("%+i%%", x), percentDelta);
    hLegend = legend(hAxes, legendLabels, Location="eastoutside");
    hLegend.Title.String = "Change from baseline";
    title(hAxes, "Corner speed deltas");
    grid(hAxes, "on");
    box(hAxes, "off");
    hAxes.XAxis.TickLength = [0, 0];

    % Accel profile
    iFig = iFig + 1;
    hFig(iFig) = figure(Name=sprintf("%s Sweep - Accel profile", thisSweepName), NumberTitle="off");
    hAxes = axes(hFig(iFig));
    hold(hAxes, "on");
    hAxes.ColorOrder = colourMap;
    for iRun = runIndices(:).'
        lap = ThisSweep.Runs(iRun).Lap;
        percentDelta = ThisSweep.Runs(iRun).PercentDelta;
        AccelProfile = getAccelProfile(lap);
        plot(hAxes, AccelProfile.vCar .* 3.6, AccelProfile.gLong ./ 9.81, ...
            DisplayName=sprintf("%+i%%", percentDelta), ...
            LineWidth=2);
    end
    xlabel(hAxes, "Speed (kph)");
    ylabel(hAxes, "Traction Acceleration (g)");
    hLegend = legend(hAxes, Location="eastoutside");
    hLegend.Title.String = "Change from baseline";
    title(hAxes, "Acceleration Profile");
    grid(hAxes, "on");
    box(hAxes, "off");

    % Braking profile
    iFig = iFig + 1;
    hFig(iFig) = figure(Name=sprintf("%s Sweep - Braking profile delta", thisSweepName), NumberTitle="off");
    hAxes = axes(hFig(iFig));
    hold(hAxes, "on");
    hAxes.ColorOrder = colourMap;
    for iRun = runIndices(:).'
        lap = ThisSweep.Runs(iRun).Lap;
        percentDelta = ThisSweep.Runs(iRun).PercentDelta;
        BrakingProfile = getBrakingProfile(lap);
        plot(hAxes, BrakingProfile.vCar .* 3.6, BrakingProfile.gLong ./ 9.81, ...
            DisplayName=sprintf("%+i%%", percentDelta), ...
            LineWidth=2);
    end
    xlabel(hAxes, "Speed (kph)");
    ylabel(hAxes, "Braking Acceleration (g)");
    hLegend = legend(hAxes, Location="eastoutside");
    hLegend.Title.String = "Change from baseline";
    title(hAxes, "Braking Profile");
    grid(hAxes, "on");
    box(hAxes, "off");
end
fontsize(hFig, 16, "points");
for ii = 1:numel(hFig)
    hFig(ii).Units = "normalized";
    hFig(ii).OuterPosition = [0.25 0.25 0.5 0.5];
end
end