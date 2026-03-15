classdef Vehicle < handle
    % Defines the parameters needed for the vehicle.
    properties
        mCarTotal               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rWeightBalF             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        wheelbase               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        hCoG                    double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        radiusTyreRollingRear   double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muTyreLong              double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muTyreLat               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rAeroBalF               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        aeroDragFactor          double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        aeroDownforceFactor     double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rFinalDrive             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rTransmissionRatio      double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        eTransmission           double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        nMotorMapLookup         (:,1) double
        MMotorMapLookup         (:,1) double
    end

    % Pre-calculated values
    properties (Access = private)
        weight double {mustBeScalarOrEmpty}
        staticWeightFrontAxle double {mustBeScalarOrEmpty}
        staticWeightRearAxle double {mustBeScalarOrEmpty}
        nMotorToMMotor {mustBeA(nMotorToMMotor, 'griddedInterpolant')}
        totalGearRatio double {mustBeScalarOrEmpty}
    end

    %% Public interface
    methods
        function initialise(this)
            this.validateParams();

            % Pre-calculated values
            g = 9.81;
            this.weight = this.mCarTotal .* g;
            this.staticWeightFrontAxle = this.rWeightBalF .* this.weight;
            this.staticWeightRearAxle = (1 - this.rWeightBalF) .* this.weight;
            this.nMotorToMMotor ...
                = griddedInterpolant(this.nMotorMapLookup, this.MMotorMapLookup, ...
                "linear", "none");
            this.totalGearRatio = this.rTransmissionRatio .* this.rFinalDrive;
        end


        function validateParams(this)
            % Since all the values are just doubles, this is a quick and dirty check to see if any numbers are invalid.
            params = metaclass(this).PropertyList;
            nParams = numel(params);
            invalidParams = string.empty();
            for ii = 1:nParams
                if strcmp(params(ii).SetAccess, 'private')
                    continue
                end

                thisParamName = params(ii).Name;
                if isempty(this.(thisParamName))
                    invalidParams = [invalidParams, convertCharsToStrings(thisParamName)]; %#ok<*AGROW>
                end
            end

            if isempty(invalidParams)
                return
            end

            invalidParams = join(invalidParams, ", ");
            error("Some parameters have not been initialised or are invalid: %s.", invalidParams);
        end


        function vCarMax = calculateMaxSteadyCornerSpeed(this, cornerRadius)
            arguments
                this Vehicle
                cornerRadius double
            end
            % TODO: Check if this function needs to take into account yaw moment balance, since the assignment brief
            % stated that it is a bicycle model. Shouldn't be hard to implement but in CW1 and all the tutorials, it was
            % never considered for the calculation of max gLat.
            % For now, we do not consider yaw moment balance.
            g = 9.81;
            vCarMax = sqrt( (this.muTyreLat .* cornerRadius .* this.mCarTotal .* g) ...
                ./ (this.mCarTotal - this.muTyreLat .* cornerRadius .* this.aeroDownforceFactor) );
        end


        function gLongMax = calculateSteadygLongTractionLimited(this, vCar, gLongCurrent)
            downforce = this.aeroDownforceFactor .* (vCar .^ 2);
            drag = this.aeroDragFactor .* (vCar .^ 2);
            FLiftF = this.rAeroBalF .* downforce;
            FLiftR = this.rAeroBalF .* downforce;
            FzFrontNoAccel = this.staticWeightFrontAxle + FLiftF;
            FzRearNoAccel = this.staticWeightRearAxle + FLiftR;
            loadTransferLong = this.mCarTotal .* gLongCurrent .* this.hCoG ./ this.wheelbase;

            % Prevent wheelies
            isLiftingFrontAxle = loadTransferLong > FzFrontNoAccel;
            loadTransferLong(isLiftingFrontAxle) = FzFrontNoAccel;

            FzRear = FzRearNoAccel + loadTransferLong;
            gLongMax = (this.muTyreLong .* FzRear - drag) ./ this.mCarTotal;
        end


        function gLongMax = calculateSteadygLongPowerLimited(this, vCar)
            nWheelRear = vCar ./ this.radiusTyreRollingRear;
            nMotor = nWheelRear .* this.totalGearRatio;
            MMotorMax = this.nMotorToMMotor(nMotor);
            MWheelMax = MMotorMax .* this.totalGearRatio;
            FAxleRearMax = MWheelMax ./ this.radiusTyreRollingRear;
            FDrag = this.aeroDragFactor .* (vCar .^ 2);
            gLongMax = (FAxleRearMax - FDrag) ./ this.mCarTotal;
        end
    end
end