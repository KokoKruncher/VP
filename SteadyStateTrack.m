classdef SteadyStateTrack < handle
    % Defines a track which is a sequence of straight -> steady-state corner -> straight -> steady-state corner -> ...
    %
    % The track starts on the first straight and can end on either a corner or on a straight. If the number of straights
    % provided is equal to the number of corners, then the track ends on the last corner. If the number of straights
    % provided is one more than the number of corners, then it ends on the last straight.
    properties
        corners (:,1) SteadyStateCorner
        straightLengths (:,1) double
        distanceStep (1,1) double = nan
        
        distanceAlongLap (:,1) double
        radius (:,1) double
        isCorner (:,1) logical
    end
    
    
    %% Constructor
    methods
        function obj = SteadyStateTrack(cornerRadii, cornerAngles, straightLengths, args)
            arguments
                cornerRadii double
                cornerAngles double
                straightLengths double
                args.DistanceStep (1,1) double = 0.1
            end
            obj.corners = SteadyStateCorner(cornerRadii, cornerAngles);
            nCorners = numel(obj.corners);
            nStraights = numel(straightLengths);
            if nStraights ~= nCorners && (nStraights - nCorners) ~= 1
                error("Invalid number of straights for the number of corners. " + ...
                    "The number of straights must be equal to or 1 more than the number of corners.")
            end
            obj.straightLengths = straightLengths;
            obj.distanceStep = args.DistanceStep;
            obj.discretiseTrack();
        end
    end
    
    
    %% Private implementation
    methods (Access = private)
        function discretiseTrack(this)
            nCorners = numel(this.corners);
            nStraights = numel(this.straightLengths);
            
            sLap = 0;
            radius = Inf; %#ok<*PROP>
            isCorner = false;
            ii = 0;
            while true
                ii = ii + 1;
                if ii > nStraights
                    break
                end
                
                previousDistance =  sLap(end);
                straightSegment = 0: this.distanceStep : this.straightLengths(ii);
                straightSegment = straightSegment + previousDistance + this.distanceStep;
                if ii <= nCorners
                    cornerSegment = 0 : this.distanceStep : this.corners(ii).distance;
                    cornerSegment = cornerSegment + straightSegment(end) + this.distanceStep;
                else
                    cornerSegment = [];
                end
                sLap = [sLap, straightSegment, cornerSegment]; %#ok<*AGROW>
                radius = [radius, Inf(size(straightSegment)), repmat(this.corners(ii).radius, size(cornerSegment))];
                isCorner = [isCorner, false(size(straightSegment)), true(size(cornerSegment))];
            end
            this.distanceAlongLap = sLap;
            this.radius = radius;
            this.isCorner = isCorner;
        end
    end
end