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
        function run(this)
            % TODO: Implement an option to run the lap from 0 speed at S/F line.
            this.vehicle.initialise();
            initialSpeedOutLap = 0;
            this.runSingleLap(initialSpeedOutLap);
        end
    end
    
    %% Private Implementation
    methods (Access = private)
        function runSingleLap(this, initialSpeed)
            % Calculate maximum corner speed.
            cornerRadii = [this.track.corners.radius];
            cornerCount = numel(cornerRadii);
            straightCount = this.track.straightCount;
            cornerSpeeds = this.vehicle.calculateMaxSteadyCornerSpeed(cornerRadii);
            iSegment = 0;
            while true
                iSegment = iSegment + 1;

            end
        end
        
        
        function forwardPass(this, distanceAlongStraight, startSpeed)
            arguments
                this
                distanceAlongStraight double {mustBeVector}
                startSpeed (1,1) double
            end
            assert(issorted(distanceAlongStraight, "ascend"), "Distance vector must be monotonically increasing.")
            nPointsAlongStraight = numel(distanceAlongStraight);
            
            vecSize = size(distanceAlongStraight);
            v = nan(vecSize);
            ax = nan(vecSize);
            ax_tractionLimited = nan(vecSize);
            ax_powerLimited = nan(vecSize);

            for ii = 1:nPointsAlongStraight                
                if ii > 1
                    prevSpeed = v(ii - 1);
                    prevAccel = ax(ii - 1);
                else
                    prevSpeed = startSpeed;
                    prevAccel = 0;
                end
                
                ax_tractionLimited(ii) = this.vehicle.calculateSteadyTractionLimitedAcceleration(prevSpeed, prevAccel);
                ax_powerLimited(ii) = this.vehicle.calculateSteadyPowerLimitedAcceleration(prevSpeed);
                ax(ii) = min(ax_tractionLimited(ii), ax_powerLimited(ii));
                
                ds = distanceAlongStraight(ii) - distanceAlongStraight(ii - 1);
                if ii > 1
                    v(ii) = sqrt(prevSpeed .^ 2 + 2 .* prevAccel .* ds);
                else
                    v(ii) = startSpeed;
                end
            end
        end
    end
end