classdef LoadDependentTireModel < handle & isConvertibleToStruct

    % Tyre model
    properties
        decayCoeff          double {mustBeScalarOrEmpty, mustBeFinite, mustBeNonnegative}
        width               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rollingRadius       double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muTyreLong_peak     double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muTyreLat_peak      double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        kSpring             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
    end

    methods
        function [muDynamic, LPUA] = calculate_tyrefrictioncoefficient(this, Fz, type)
            if strcmp(type, 'long')
                mu_peak = this.muTyreLong_peak;
            elseif strcmp(type, 'lat')
                mu_peak = this.muTyreLat_peak;
            else
                error("Enter either 'lat' or 'long' as type.")
            end

            LPUA = Fz./(this.width.*this.rollingRadius.*pi);
            muDynamic = mu_peak - (this.decayCoeff.*mu_peak.*LPUA);
            
           % Minimum mu is zero
           muDynamic(muDynamic < 0) = 0;
        end
    end

end