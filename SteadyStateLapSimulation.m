classdef SteadyStateLapSimulation < handle
    properties
        track SteadyStateTrack {mustBeScalarOrEmpty}
        vehicle Vehicle {mustBeScalarOrEmpty}
        distanceStep double {mustBeScalarOrEmpty}
    end
    
    
    %% Constructor
    methods
        function obj = SteadyStateLapSimulation(track, vehicle, args)
            arguments
                track (1,1)
                vehicle (1,1)
                args.distanceStep (1,1) double = 0.1
            end
            obj.track = track;
            obj.vehicle = vehicle;
            obj.distanceStep = args.distanceStep;
        end
    end
    
    %% Public interface
    methods
        function states = run(this)
            this.vehicle.initialise();
            
            % Out lap
            vCarInitialOutLap = 0;
            states = this.runSingleLap(vCarInitialOutLap);
            
            % Push lap
            % TODO: Optimise further by considering whether only first corner needs to be rerun 
            vCarInitialPushLap = states.results.vCar(end);
            states = this.runSingleLap(vCarInitialPushLap);
        end
    end
    
    %% Private Implementation
    methods (Access = private)
        function [lap, straightStates, cornerStates] = runSingleLap(this, vCarInitial, args)
            straightCount = this.track.straightCount;
            cornerCount = numel(this.track.corners);
            
            % Internal state objects.
            straightStates_(straightCount) = VehicleStates();
            cornerStates_(straightCount) = VehicleStates();
            
            % Initialise array to hold maximum corner speed for backtracking if braking zone not long enough.
            maxCornerSpeed = inf(1, cornerCount);
            
            iSegment = 0;
            while true
                iSegment = iSegment + 1;
                if iSegment > straightCount
                    break
                end
                
                if iSegment > 1
                    vCarStartOfStraight = straightStates_(iSegment - 1).results.vCar(end);
                else
                    vCarStartOfStraight = vCarInitial;
                end
                
                sRunStraight = 0 : this.distanceStep : this.track.straightLengths(iSegment);
                forwardPass = this.vehicle.driveStraightLineAccel(sRunStraight, vCarStartOfStraight);
                if iSegment > cornerCount
                    straightStates_(iSegment) = forwardPass;
                    break
                end
                
                % Find out max speed into next corner
                sRunCorner = 0: this.distanceStep : this.track.corners(iSegment).distance;
                radiusCorner = this.track.corners(iSegment).radius;
                corner = this.vehicle.driveSteadyStateCorner(sRunCorner, radiusCorner);
                vCarEndOfStraight = corner.results.vCar(1);
                
                backwardsPass = this.vehicle.driveStraightLineBraking(sRunStraight, vCarEndOfStraight);
                
                if backwardsPass.results.vCar(1) < vCarStartOfStraight
                    % Previous corner too fast
                    % TODO: Handle previous corner too fast scenario.
                    error("Unhandled scenario: Previous corner too fast.")
                elseif forwardPass.results.vCar(end) < vCarEndOfStraight
                    % Next corner is too fast, there is not enough straight to accelerate to the maximum corner speed.
                    warning("Not enough straight to accelerate to maximum corner speed for next corner.")
                    nextCornerSpeedLimit = forwardPass.results.vCar(end);
                    corner = this.vehicle.driveSteadyStateCornerAtSpeed(sRunCorner, radiusCorner, nextCornerSpeedLimit);
                    straightStates_(iSegment) = forwardPass;
                    cornerStates_(iSegment) = corner;
                else
                    % Regular scenario
                    iStartBraking = find(forwardPass.results.vCar > backwardsPass.results.vCar, 1, "first");
                    forwardPass.crop(EndIndex=iStartBraking - 1);
                    backwardsPass.crop(StartIndex=iStartBraking);
                    straightStates_(iSegment) = forwardPass.append(backwardsPass);
                    cornerStates_(iSegment) = corner;
                end
            end
            
            % These internal states need to be copied to be output as they are handle objects which get modified.
            straightStates = straightStates_.copy();
            cornerStates = cornerStates_.copy();
            
            % Assemble the full lap
            lap = straightStates_(1);
            for ii = 1:cornerCount
                lap.append(cornerStates_(ii));
                if ii + 1 > straightCount
                    break
                end
                lap.append(straightStates_(ii + 1));
            end
        end
    end
end