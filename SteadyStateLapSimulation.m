classdef SteadyStateLapSimulation < handle
    properties
        track SteadyStateTrack {mustBeScalarOrEmpty}
        vehicle Vehicle {mustBeScalarOrEmpty}
        distanceStep double {mustBeScalarOrEmpty}
        enableLoggingToCommandWindow (1,1) logical = false
    end
    
    
    %% Constructor
    methods
        function obj = SteadyStateLapSimulation(track, vehicle, args)
            arguments
                track (1,1)
                vehicle (1,1)
                args.distanceStep (1,1) double = 0.1
                args.enableLoggingToCommandWindow (1,1) logical = false
            end
            obj.track = track;
            obj.vehicle = vehicle;
            obj.distanceStep = args.distanceStep;
            obj.enableLoggingToCommandWindow = args.enableLoggingToCommandWindow;
        end
    end
    
    %% Public interface
    methods
        function lap = run(this)
            MAX_NUM_PUSH_LAP_RERUNS = 10;
            VCAR_START_AND_END_OF_LAP_TOLERENCE = 0.01;
            
            tSimulationTotal = 0;
            
            % Out lap to get speed at start-finish line.
            this.vehicle.initialise();
            this.log("=== Running out lap ===")
            vCarInitialOutLap = 0;
            [lap, tSimulation] = this.runSingleLap(vCarInitialOutLap);
            tSimulationTotal = tSimulationTotal + tSimulation;
            
            % Push lap
            % TODO: Optimise further by considering whether only first corner needs to be rerun 
            % DELETE THIS
            iPushLap = 0;
            while true
                iPushLap = iPushLap + 1;
                if iPushLap > MAX_NUM_PUSH_LAP_RERUNS
                    warning("Max number of push lap reruns reached. Lap may not be converged!");
                    break
                end
                vCarInitial = lap.results.vCar(end);
                this.log(sprintf("=== Running push lap iteration %i ===", iPushLap), PrependNewlines=1);
                [lap, tSimulation] = this.runSingleLap(vCarInitial);
                tSimulationTotal = tSimulationTotal + tSimulation;
                
                if abs(lap.results.vCar(end) - lap.results.vCar(1)) < VCAR_START_AND_END_OF_LAP_TOLERENCE
                    break
                end
            end
            
            this.log("Lap simulation finished.", PrependNewlines=1);
            this.log(sprintf("Simulation time = %.2fs", tSimulationTotal));
        end
    end
    
    %% Private Implementation
    methods (Access = private)
        function [lap, tSimulation] = runSingleLap(this, vCarInitial)
            tic
            MAX_NUM_BACKTRACK_RERUNS = 10;
            straightCount = this.track.straightCount;
            cornerCount = numel(this.track.corners);
            
            % Internal state objects.
            straights(straightCount) = VehicleStates();
            corners(straightCount) = VehicleStates();
            
            % Initialise arrays to hold information needed for backtracking if the braking zone is not long enough to
            % make the next corner.
            maxCornerSpeed = inf(1, cornerCount);
            isBacktrackNeeded = false(1, cornerCount);
            
            % Since track starts with a straight and could end on either a straight or a corner, a segment is considered
            % as a pair of 1 straight and 1 corner, or just 1 straight and no corner.
            iSegment = 0;
            while true
                iSegment = iSegment + 1;
                if iSegment > straightCount
                    break
                end
                
                if iSegment > 1
                    vCarStartOfStraight = straights(iSegment - 1).results.vCar(end);
                else
                    vCarStartOfStraight = vCarInitial;
                end
                
                sRunStraight = 0 : this.distanceStep : this.track.straightLengths(iSegment);
                forwardPass = this.vehicle.driveStraightLineAccel(sRunStraight, vCarStartOfStraight);
                if iSegment > cornerCount
                    straights(iSegment) = forwardPass;
                    break
                end
                
                % Find out max speed into next corner
                sRunCorner = 0: this.distanceStep : this.track.corners(iSegment).distance;
                radiusCorner = this.track.corners(iSegment).radius;
                corner = this.vehicle.driveSteadyStateCorner(sRunCorner, radiusCorner);
                vCarEndOfStraight = corner.results.vCar(1);
                
                backwardsPass = this.vehicle.driveStraightLineBraking(sRunStraight, vCarEndOfStraight);
                if backwardsPass.results.vCar(1) < vCarStartOfStraight
                    % Previous corner was too fast, there is not enough distance to brake for the next corner.
                    % Previous segment now needs to be rerun with max corner speed = start of straight speed of this
                    % segment.
                    this.log(sprintf( ...
                        "Not enough distance to decelerate for corner %i. Backtracking needed.", iSegment));
                    
                    straights(iSegment) = backwardsPass;
                    corners(iSegment) = corner;
                    if iSegment > 1
                        iPreviousSegment = iSegment - 1;
                    else 
                        % Rerun last corner
                        iPreviousSegment = cornerCount;
                    end
                    maxCornerSpeed(iPreviousSegment) = backwardsPass.results.vCar(1);
                    isBacktrackNeeded(iPreviousSegment) = true;
                elseif forwardPass.results.vCar(end) < vCarEndOfStraight
                    % Next corner is too fast, there is not enough straight to accelerate to the maximum corner speed.
                    this.log(sprintf( ...
                        "Not enough distance to accelerate to corner %i max speed. Corner speed reduced.", iSegment));
                   
                    nextCornerSpeedLimit = forwardPass.results.vCar(end);
                    corner = this.vehicle.driveSteadyStateCornerAtSpeed(sRunCorner, radiusCorner, nextCornerSpeedLimit);
                    straights(iSegment) = forwardPass;
                    corners(iSegment) = corner;
                else
                    % Regular scenario
                    iStartBraking = find(forwardPass.results.vCar > backwardsPass.results.vCar, 1, "first");
                    forwardPass.crop(EndIndex=iStartBraking - 1);
                    backwardsPass.crop(StartIndex=iStartBraking);
                    straights(iSegment) = forwardPass.append(backwardsPass);
                    corners(iSegment) = corner;
                end
            end
            
            % Backtrack
            backtrackRerunCount = 0;
            while true
                if ~any(isBacktrackNeeded)
                    break
                end
                
                backtrackRerunCount = backtrackRerunCount + 1;
                if backtrackRerunCount > MAX_NUM_BACKTRACK_RERUNS
                    warning("Maximum number of backtracks reached. Lap may not have converged!");
                    break
                end
                this.log(sprintf("Backtracking, iteration %i", backtrackRerunCount));
                [straights, corners, isBacktrackNeeded, maxCornerSpeed] = ...
                    this.backtrack(straights, corners, isBacktrackNeeded, maxCornerSpeed);
            end
            
            % Assemble the full lap
            lap = straights(1);
            for ii = 1:cornerCount
                lap.append(corners(ii));
                if ii + 1 > straightCount
                    break
                end
                lap.append(straights(ii + 1));
            end
            
            tSimulation = toc;
        end
        
        
        function [straights, corners, isBacktrackNeededNext, maxCornerSpeedsNext] = ...
                backtrack(this, straights, corners, isBacktrackNeeded, maxCornerSpeeds)
            
            % Update these if another iteration of backtracking is needed.
            isBacktrackNeededNext = false(size(isBacktrackNeeded));
            maxCornerSpeedsNext = maxCornerSpeeds;
            
            cornerCount = numel(corners);
            iBacktrackNeeded = find(isBacktrackNeeded);
            for iSegment = iBacktrackNeeded(:).'
                if iSegment > 1
                    vCarStartOfStraight = straights(iSegment - 1).results.vCar(end);
                else
                    vCarStartOfStraight = straights(1).results.vCar(1);
                end
                
                sRunStraight = 0 : this.distanceStep : this.track.straightLengths(iSegment);
                forwardPass = this.vehicle.driveStraightLineAccel(sRunStraight, vCarStartOfStraight);
                
                % Limit the maximum corner speed
                vCarEndOfStraight = maxCornerSpeeds(iSegment);
                backwardsPass = this.vehicle.driveStraightLineBraking(sRunStraight, vCarEndOfStraight);
                sRunCorner = corners(iSegment).results.sRun;
                radiusCorner = this.track.corners(iSegment).radius;
                corner = this.vehicle.driveSteadyStateCornerAtSpeed(sRunCorner, radiusCorner, vCarEndOfStraight);
                
                if forwardPass.results.vCar(end) < vCarEndOfStraight
                    % Somehow we messed up the other scenario too???
                    error("Error while backtracking. Straight too short to accelerate to next corner speed. " + ...
                        "Lap may not have converged!");
                end
                
                if backwardsPass.results.vCar(1) < vCarStartOfStraight
                    % Need even more backtracking
                    straights(iSegment) = backwardsPass;
                    corners(iSegment) = corner;
                    if iSegment > 1
                        iPreviousSegment = iSegment - 1;
                    else
                        iPreviousSegment = cornerCount;
                    end
                    isBacktrackNeededNext(iPreviousSegment) = true;
                    maxCornerSpeedsNext(iPreviousSegment) = backwardsPass.results.vCar(1);
                else
                    iStartBraking = find(forwardPass.results.vCar > backwardsPass.results.vCar, 1, "first");
                    forwardPass.crop(EndIndex=iStartBraking - 1);
                    backwardsPass.crop(StartIndex=iStartBraking);
                    straights(iSegment) = forwardPass.append(backwardsPass);
                    corners(iSegment) = corner;
                end
            end
        end
        
        
        function log(this, txt, args)
            arguments
                this 
                txt {mustBeTextScalar}
                args.PrependNewlines (1,1) double {mustBeInteger, mustBeNonnegative} = 0;
            end
            if ~this.enableLoggingToCommandWindow
                return
            end
            if args.PrependNewlines == 0
                prependedNewlines = "";
            else
                prependedNewlines = join(repmat("\n", 1, args.PrependNewlines), "");
            end
            fprintf(1, prependedNewlines + "LOG: %s\n", txt);
        end
    end
end