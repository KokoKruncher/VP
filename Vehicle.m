classdef Vehicle < handle
    % Defines the parameters needed for the vehicle.
    properties
        mass                            double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        weightDistribution              double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        wheelbase                       double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        cogHeight                       double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rearTyreRollingRadius           double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muLongitudinal                  double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muLateral                       double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        aeroBalance                     double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        dragFactor                      double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        downforceFactor                 double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        finalDriveRatio                 double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        gearRatio                       double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        drivelineEfficiency             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        motorTorqueLookup_RevsPerSecond (:,1) double
        motorTorqueLookup_Torque        (:,1) double
    end

    % Pre-calculated values
    properties (Access = private)
        weight double {mustBeScalarOrEmpty}
        staticWeightFrontAxle double {mustBeScalarOrEmpty}
        staticWeightRearAxle double {mustBeScalarOrEmpty}
        motorRevsPerSecondToTorque {mustBeA(motorRevsPerSecondToTorque, 'griddedInterpolant')}
        totalGearRatio double {mustBeScalarOrEmpty}
    end

    %% Public interface
    methods
        function initialise(this)
            this.validateParams();

            % Pre-calculated values
            g = 9.81;
            this.weight = this.mass .* g;
            this.staticWeightFrontAxle = this.weightDistribution .* this.weight;
            this.staticWeightRearAxle = (1 - this.weightDistribution) .* this.weight;
            this.motorRevsPerSecondToTorque ...
                = griddedInterpolant(this.motorTorqueLookup_RevsPerSecond, this.motorTorqueLookup_Torque, ...
                "linear", "none");
            this.totalGearRatio = this.gearRatio .* this.finalDriveRatio;
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


        function vMax = calculateMaxSteadyCornerSpeed(this, cornerRadius)
            arguments
                this Vehicle
                cornerRadius double
            end
            % TODO: Check if this function needs to take into account yaw moment balance, since the assignment brief
            % stated that it is a bicycle model. Shouldn't be hard to implement but in CW1 and all the tutorials, it was
            % never considered for the calculation of max gLat.
            % For now, we do not consider yaw moment balance.
            g = 9.81;
            vMax = sqrt( (this.muLateral .* cornerRadius .* this.mass .* g) ...
                ./ (this.mass - this.muLateral .* cornerRadius .* this.downforceFactor) );
        end


        function axMax = calculateSteadyTractionLimitedAcceleration(this, speed, currentLongitudinalAcceleration)
            downforce = this.downforceFactor .* (speed .^ 2);
            drag = this.dragFactor .* (speed .^ 2);
            frontDownforce = this.aeroBalance .* downforce;
            rearDownforce = this.aeroBalance .* downforce;
            frontAccelVerticalLoadNoAccel = this.staticWeightFrontAxle + frontDownforce;
            rearAccelVerticalLoadNoAccel = this.staticWeightRearAxle + rearDownforce;
            longitudinalLoadTransfer = this.mass .* currentLongitudinalAcceleration .* this.cogHeight ./ this.wheelbase;

            % Prevent wheelies
            isLiftingFrontAxle = longitudinalLoadTransfer > frontAccelVerticalLoadNoAccel;
            longitudinalLoadTransfer(isLiftingFrontAxle) = frontAccelVerticalLoadNoAccel;

            rearAccelVerticalLoad = rearAccelVerticalLoadNoAccel + longitudinalLoadTransfer;
            axMax = (this.muLongitudinal .* rearAccelVerticalLoad - drag) ./ this.mass;
        end


        function axMax = calculateSteadyPowerLimitedAcceleration(this, speed)
            wheelSpeed = speed ./ this.rearTyreRollingRadius;
            motorSpeed = wheelSpeed .* this.totalGearRatio;
            motorMaxTorque = this.motorRevsPerSecondToTorque(motorSpeed);
            wheelMaxTorque = motorMaxTorque .* this.totalGearRatio;
            maxTractionForce = wheelMaxTorque ./ this.rearTyreRollingRadius;
            drag = this.dragFactor .* (speed .^ 2);
            axMax = (maxTractionForce - drag) ./ this.mass;
        end
    end
end