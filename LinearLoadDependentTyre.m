classdef LinearLoadDependentTyre < Tyre
    properties
        peakMuLon           double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        peakMuLat           double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muDecayCoeffient    double {mustBeScalarOrEmpty, mustBeFinite} % Per unit vertical load
    end
    
    %% Constructor
    methods
        function obj = LinearLoadDependentTyre(rollingRadius, peakMuLon, peakMuLat, muDecayCoefficient)
            obj.rollingRadius = rollingRadius;
            obj.peakMuLon = peakMuLon;
            obj.peakMuLat = peakMuLat;
            obj.muDecayCoeffient = muDecayCoefficient;
        end
    end
    
    %% Public interface
    methods
        function FxMax = calculateFxMax(this, Fz)
            FxMax = Fz .* this.calculateMuDynamic(this.peakMuLon, Fz);
        end
        
        
        function FyMax = calculateFyMax(this, Fz)
            FyMax = Fz .* this.calculateMuDynamic(this.peakMuLat, Fz);
        end
    end
    
    %% Private implementation
    methods (Access = private)
        function muDynamic = calculateMuDynamic(this, muPeak, Fz)
            muDynamic = muPeak - (this.muDecayCoeffient .* Fz .* muPeak);
            muDynamic(Fz <= 0) = 0;
        end
    end
end