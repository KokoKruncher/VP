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
            % TODO: start simulating from the last corner
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
        function states = runSingleLap(this, vCarInitial)
            tic
            % Calculate maximum corner speed.
            cornerCount = numel(this.track.corners);
            straightCount = this.track.straightCount;
            iSegment = 0;
            states = VehicleStates();
            while true
                iSegment = iSegment + 1;
                if iSegment > straightCount
                    break
                end
                
                if iSegment > 1
                    vCarStartOfStraight = states.results.vCar(end);
                else
                    vCarStartOfStraight = vCarInitial;
                end
                
                sRunStraight = 0 : this.distanceStep : this.track.straightLengths(iSegment);
                forwardPass = this.vehicle.driveStraightLineAccel(sRunStraight, vCarStartOfStraight);
                if iSegment > cornerCount
                    states.append(forwardPass);
                    break
                end
                
                % Find out max speed into next corner
                sRunCorner = 0: this.distanceStep : this.track.corners(iSegment).distance;
                radiusCorner = this.track.corners(iSegment).radius;
                corner = this.vehicle.driveSteadyStateCorner(sRunCorner, radiusCorner);
                vCarEndOfStraight = corner.results.vCar(1);
                
                backwardsPass = this.vehicle.driveStraightLineBraking(sRunStraight, vCarEndOfStraight);
                iStartBraking = find(forwardPass.results.vCar > backwardsPass.results.vCar, 1, "first");
                if iStartBraking == 1
                    % Previous corner too fast
                    % TODO: Handle previous corner too fast scenario.
                    error("Unhandled scenario: Previous corner too fast.")
                elseif isempty(iStartBraking)
                    % Previous corner too fast (next corner too fast)
                    % TODO: Handle next corner too slow scenatio.
                    error("Unhandled scenario : Next corner too slow.")
                end
                
                forwardPass.crop(EndIndex=iStartBraking - 1);
                backwardsPass.crop(StartIndex=iStartBraking);
                states.append(forwardPass).append(backwardsPass).append(corner);
            end
            toc
        end
    end
end