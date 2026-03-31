classdef Vehicle < handle
    % A vehicle that can perform simple steady-state manouvres.
    properties
        mCarTotal               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rWeightBalF             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        wheelbase               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        hCoG                    double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        radiusTyreRollingRear   double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        TyreWidthFront          double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        TyreWidthRear           double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muTyreLong_peak         double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        muTyreLat_peak          double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        tyreDecayCoefficient    double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rAeroBalF               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        aeroDragFactor          double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        aeroDownforceFactor     double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rFinalDrive             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rTransmissionRatio      double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        eTransmission           double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        nMotorMapLookup         (:,1) double % radians per second
        MMotorMapLookup         (:,1) double
    end

    % Pre-calculated values
    properties (Access = private)
        weight double {mustBeScalarOrEmpty}
        FzFrontStatic double {mustBeScalarOrEmpty}
        FzRearStatic double {mustBeScalarOrEmpty}
        nMotorToMMotorMax {mustBeA(nMotorToMMotorMax, 'griddedInterpolant')} = griddedInterpolant()
        totalGearRatio double {mustBeScalarOrEmpty}
    end

    %% Public interface
    methods
        function initialise(this)
            this.validateParams();

            % Pre-calculated values
            g = 9.81;
            this.weight = this.mCarTotal .* g;
            this.FzFrontStatic = this.rWeightBalF .* this.weight;
            this.FzRearStatic = (1 - this.rWeightBalF) .* this.weight;
            this.nMotorToMMotorMax ...
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


        function state = driveSteadyStateCorner(this, sRun, cornerRadius)
            arguments
                this Vehicle
                sRun
                cornerRadius (1,1) double
            end
            % TODO: Check if this function needs to take into account yaw moment balance, since the assignment brief
            % stated that it is a bicycle model. Shouldn't be hard to implement but in CW1 and all the tutorials, it was
            % never considered for the calculation of max gLat.
            % For now, we do not consider yaw moment balance.
            g = 9.81;
            gLong = 0;...
            vCar = sqrt( (this.muTyreLat_peak .* cornerRadius .* this.mCarTotal .* g) ...
                      ./ (this.mCarTotal - this.muTyreLat_peak .* cornerRadius .* this.aeroDownforceFactor) );
            gLat = this.calculate_gLat(vCar, cornerRadius);

            state = VehicleStates(sRun);
            state.logConstant("vCar", vCar);
            state.logConstant("gLong", gLong);
            state.logConstant("gLat", gLat);
            this.backCalculateStates(state);

            [muDynamicFront, LPUAF] = this.calculate_tyrefrictioncoefficient(state.results.FzFront, 'lat', 'front');
            [muDynamicRear, LPUAR]  = this.calculate_tyrefrictioncoefficient(state.results.FzRear, 'lat', 'rear');
            indices = 1:numel(sRun);
            state.log("LPUAF", LPUAF, indices);
            state.log("dymuF", muDynamicFront, indices);
            state.log("LPUAR", LPUAR, indices);
            state.log("dymuR", muDynamicRear, indices);
        end


        function state = driveStraightLineAccel(this, sRun, vCarStartOfStraight)
            arguments
                this Vehicle
                sRun (:,1) double
                vCarStartOfStraight (1,1) double
            end
            assert(issorted(sRun, "strictascend"), "Distance vector must be monotonically increasing.")
            stepCount = numel(sRun);
            vecSize = size(sRun);
            vCar = nan(vecSize);
            gLong = nan(vecSize);
            gLongTractionLimited = nan(vecSize);
            gLongPowerLimited = nan(vecSize);

            for ii = 1:stepCount                
                if ii > 1
                    vCarPrev = vCar(ii - 1);
                    gLongPrev = gLong(ii - 1);
                else
                    vCarPrev = vCarStartOfStraight;
                    gLongPrev = 0;
                end
                
                gLongTractionLimited(ii) = this.calculate_gLongTractionLimitedStraightLine(vCarPrev, gLongPrev);
                gLongPowerLimited(ii) = this.calculate_gLongPowerLimited(vCarPrev);
                gLong(ii) = min(gLongTractionLimited(ii), gLongPowerLimited(ii));
                
                if ii > 1
                    ds = sRun(ii) - sRun(ii - 1);
                    vCar(ii) = sqrt(vCarPrev .^ 2 + 2 .* gLongPrev .* ds);
                else
                    vCar(ii) = vCarStartOfStraight;
                end
            end
            
            state = VehicleStates(sRun);
            indices = 1:numel(sRun);
            state.log("vCar", vCar, indices);
            state.logConstant("gLat", 0);
            state.log("gLong", gLong, indices);
            state.log("gLongTractionLimited", gLongTractionLimited, indices);
            state.log("gLongPowerLimited", gLongPowerLimited, indices);
            this.backCalculateStates(state);

            [muDynamicRear, LPUAR] = this.calculate_tyrefrictioncoefficient(state.results.FzRear, 'long', 'rear');
            state.log("LPUAR", LPUAR, indices);
            state.log("dymuR", muDynamicRear, indices);
        end
        
        
        function state = driveStraightLineBraking(this, sRun, vCarEndOfStraight)
            arguments
                this 
                sRun (:,1) double
                vCarEndOfStraight (1,1) double
            end
            assert(issorted(sRun, "strictascend"), "Distance vector must be monotonically increasing.")
            stepCount = numel(sRun);
            vecSize = size(sRun);
            vCar = nan(vecSize);
            gLong = nan(vecSize);

            for ii = stepCount : -1 : 1              
                if ii < stepCount
                    vCarNext = vCar(ii + 1);
                    gLongNext = gLong(ii + 1);
                else
                    vCarNext = vCarEndOfStraight;
                    gLongNext = 0;
                end
                
                gLong(ii) = this.calculate_gLongBraking(vCarNext, gLongNext);
                
                if ii < stepCount
                    ds = sRun(ii + 1) - sRun(ii);
                    vCar(ii) = sqrt(vCarNext .^ 2 - 2 .* gLong(ii) .* ds);
                else
                    vCar(ii) = vCarEndOfStraight;
                end
            end
            
            state = VehicleStates(sRun);
            indices = 1:numel(sRun);
            state.log("vCar", vCar, indices);
            state.logConstant("gLat", 0);
            state.log("gLong", gLong, indices);
            this.backCalculateStates(state);

            [muDynamicFront, LPUAF] = this.calculate_tyrefrictioncoefficient(state.results.FzFront, 'long', 'front');
            [muDynamicRear, LPUAR]  = this.calculate_tyrefrictioncoefficient(state.results.FzRear, 'long', 'rear');
            state.log("LPUAF", LPUAF, indices);
            state.log("dymuF", muDynamicFront, indices);
            state.log("LPUAR", LPUAR, indices);
            state.log("dymuR", muDynamicRear, indices);
        end
    end
    
    %% Private implementation
    methods (Access = private, Static)
        function tRun = calculate_tRun(sRun, vCar)
            tRun = cumtrapz(sRun, 1 ./ vCar);
        end
        
        
        function gLat = calculate_gLat(vCar, radiusCorner)
            gLat = (vCar .^ 2) ./ radiusCorner;
        end
    end
    
    
    methods (Access = private)
        function backCalculateStates(this, state)
            arguments
                this 
                state (1,1) VehicleStates 
            end
            % Requires vCar, gLat & gLong to be pre-calculated.
            tRun = this.calculate_tRun(state.results.sRun, state.results.vCar);
            [FLiftF, FLiftR, FDrag] = this.calculateAeroLoads(state.results.vCar);
            [FzFront, FzRear] = this.calculateAxleLoads(state.results.vCar, state.results.gLong);
            FxTyreRear = this.mCarTotal .* state.results.gLong;
            MMotor = this.calculate_MMotorFromFxTyreRear(FxTyreRear);
            
            % Motor only supplies power in traction, no regen considered.
            MMotor(MMotor < 0) = 0;
            
            indices = 1:numel(tRun);
            state.log("tRun", tRun, indices);
            state.log("MMotor", MMotor, indices);
            state.log("FzFront", FzFront, indices);
            state.log("FzRear", FzRear, indices);
            state.log("FLiftF", FLiftF, indices);
            state.log("FLiftR", FLiftR, indices);
            state.log("FDrag", FDrag, indices);         
        end
        
        
        function [FLiftF, FLiftR, FDrag] = calculateAeroLoads(this, vCar)
            FLiftTotal = this.aeroDownforceFactor .* (vCar .^ 2);
            FDrag = this.aeroDragFactor .* (vCar .^2);
            FLiftF = this.rAeroBalF .* FLiftTotal;
            FLiftR = (1 - this.rAeroBalF) .* FLiftTotal;
        end
        
        
        function [FzFront, FzRear] = calculateAxleLoads(this, vCar, gLong)
            [FLiftF, FLiftR, ~] = this.calculateAeroLoads(vCar);
            loadTransferLong = (this.mCarTotal .* gLong .* this.hCoG) ./ this.wheelbase;
            FzFront = this.FzFrontStatic + FLiftF - loadTransferLong;
            FzRear = this.FzRearStatic + FLiftR + loadTransferLong;
        end
        

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
            muDynamic = mu_peak - (this.tyreDecayCoefficient.*mu_peak.*LPUA);
        end


        function MMotor = calculate_MMotorFromFxTyreRear(this, FxTyreRear)
            MWheel = FxTyreRear .* this.radiusTyreRollingRear;
            MMotor = MWheel ./ this.totalGearRatio;
            MMotor = MMotor ./ this.eTransmission;
        end
        
        
        function gLongTractionLimited = calculate_gLongTractionLimitedStraightLine(this, vCar, gLongPrev)
            [FzFront, FzRear] = this.calculateAxleLoads(vCar, gLongPrev);
            % Prevent wheelies
            FzFrontNegative = FzFront;
            FzFrontNegative(FzFrontNegative >= 0) = 0;

            % Dynamic friction coefficient
            muDynamic = this.calculate_tyrefrictioncoefficient(FzRear, 'long', 'rear');

            % FzFront = FzFront - FzFrontNegative;
            FzRear = FzRear + FzFrontNegative;
            gLongTractionLimited = FzRear .* muDynamic ./ this.mCarTotal;
        end


        function gLongPowerLimited = calculate_gLongPowerLimited(this, vCar)
            nWheelRear = vCar ./ this.radiusTyreRollingRear;
            nMotor = nWheelRear .* this.totalGearRatio;
            MMotorMax = this.nMotorToMMotorMax(nMotor);
            if isnan(MMotorMax)
                % We are outside the motor torque map. We will assume that the last nMotor value in the map is the max
                % rpm of the motor and give 0 power here.
                MMotorMax = 0;
            end
            MWheelMax = MMotorMax .* this.totalGearRatio .* this.eTransmission;
            FxTyreRearPowerLimited = MWheelMax ./ this.radiusTyreRollingRear;
            [~, ~, FDrag] = this.calculateAeroLoads(vCar);
            gLongPowerLimited = (FxTyreRearPowerLimited - FDrag) ./ this.mCarTotal;
        end
        
        
        function gLongBraking = calculate_gLongBraking(this, vCar, gLongNext)
            [FzFront, FzRear] = this.calculateAxleLoads(vCar, gLongNext);
            muDynamicFront = this.calculate_tyrefrictioncoefficient(FzFront, 'long', 'front');
            muDynamicRear  = this.calculate_tyrefrictioncoefficient(FzRear, 'long', 'rear');
            [~, ~, FDrag] = this.calculateAeroLoads(vCar);
            gLongBraking = (-(muDynamicFront.*FzFront + muDynamicRear.*FzRear) - FDrag) ./ this.mCarTotal;
        end
    end
end