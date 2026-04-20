classdef LoadDependentTireModel < handle

    % Tyre model
    properties
        tyreDecayCoeff          double {mustBeScalarOrEmpty, mustBeFinite}
        TyreWidthFront          double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        TyreWidthRear           double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        radiusTyreRollingRear   double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}

        muTyreLong_peak         double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muTyreLat_peak          double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
    end

    methods
        function [muDynamic, LPUA] = calculate_tyrefrictioncoefficient(this, Fz, type, axle)
            if strcmp(axle, 'front')
                TyreWidth = this.TyreWidthFront;
            else
                TyreWidth = this.TyreWidthRear;
            end

            if strcmp(type, 'long')
                mu_peak = this.muTyreLong_peak;
            else
                mu_peak = this.muTyreLat_peak;
            end

            LPUA = Fz./(TyreWidth.*this.radiusTyreRollingRear.*pi);
            muDynamic = mu_peak - (this.tyreDecayCoeff.*mu_peak.*LPUA);
        end
    end

end