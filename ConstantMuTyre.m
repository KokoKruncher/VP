classdef ConstantMuTyre < Tyre
    properties
        muLong  double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muLat   double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
    end
    
    %% Constructor
    methods
        function obj = ConstantMuTyre(rollingRadius, muLong, muLat)
            obj.rollingRadius = rollingRadius;
            obj.muLong = muLong;
            obj.muLat = muLat;
        end
    end
    
    %% Public interface
    methods
        function FxMax = calculateFxMax(this, Fz)
            FxMax = this.muLong .* Fz;
            FxMax(Fz <= 0) = 0;
        end
        
        function FyMax = calculateFyMax(this, Fz)
            FyMax = this.muLat .* Fz;
            FyMax(Fz <= 0) = 0;
        end
    end
end