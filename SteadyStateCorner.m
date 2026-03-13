classdef SteadyStateCorner < handle
    properties
        radius (1,1) double = nan
        angle (1,1) double = nan
        distance (1,1) double = nan
    end
    
    
    methods
        function obj = SteadyStateCorner(radii, angles)
            arguments
                radii = []
                angles = []
            end
            nRadii = numel(radii);
            nAngles = numel(angles);
            assert(nRadii == nAngles, "Number of corner radii and angles must be equal.")
            
            nCorners = nRadii;
            if nCorners < 1
                % obj = SteadyStateCorner.empty();
                return
            end
            
            if nCorners == 1
                obj.radius = radii;
                obj.angle = angles;
                obj.distance = radii .* angles;
                return
            end
            
            for ii = nCorners:-1:1
                obj(ii) = SteadyStateCorner(radii(ii), angles(ii));
            end
        end
    end
end