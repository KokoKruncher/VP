classdef Vehicle < handle
    % A vehicle that can perform simple steady-state manouvres.
    properties
        mCarTotal               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rWeightBalF             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        wheelbase               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        hCoG                    double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rAeroBalF               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        aeroDragFactor          double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        aeroDownforceFactor     double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rFinalDrive             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rTransmissionRatio      double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        eTransmission           double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        nMotorMapLookup         (:,1) double % radians per second
        MMotorMapLookup         (:,1) double
        tire                    LoadDependentTireModel
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
            g = 9.81;
            gLong = 0;

            % Compute maximum achievable speed for this radius
            vCar = this.solveMaxSpeedInCorner(cornerRadius);
            gLat = this.calculate_gLat(vCar, cornerRadius);

            state = VehicleStates(sRun);
            state.logConstant("vCar", vCar);
            state.logConstant("gLong", gLong);
            state.logConstant("gLat", gLat);
            this.backCalculateStates(state);

            % Log dynamic friction coefficients (per axle, using per‑tire loads)
            [muDynamicFront, LPUAF] = this.tire.calculate_tyrefrictioncoefficient(...
                state.results.FzFront/2, 'lat', 'front');
            [muDynamicRear, LPUAR]  = this.tire.calculate_tyrefrictioncoefficient(...
                state.results.FzRear/2, 'lat', 'rear');
            indices = 1:numel(sRun);
            state.log("LPUAF", LPUAF, indices);
            state.log("muDynamicF", muDynamicFront, indices);
            state.log("LPUAR", LPUAR, indices);
            state.log("muDynamicR", muDynamicRear, indices);
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

            [muDynamicRear, LPUAR] = this.tire.calculate_tyrefrictioncoefficient(state.results.FzRear./2, 'long', 'rear');
            state.log("LPUAR", LPUAR, indices);
            state.log("muDynamicR", muDynamicRear, indices);
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

            [muDynamicFront, LPUAF] = this.tire.calculate_tyrefrictioncoefficient(state.results.FzFront./2, 'long', 'front');
            [muDynamicRear, LPUAR]  = this.tire.calculate_tyrefrictioncoefficient(state.results.FzRear./2, 'long', 'rear');
            state.log("LPUAF", LPUAF, indices);
            state.log("muDynamicF", muDynamicFront, indices);
            state.log("LPUAR", LPUAR, indices);
            state.log("muDynamicR", muDynamicRear, indices);
        end
        
        
        function testNewtonRaphson(this)
            fcn = @(x) x - 5;
            x0 = 2;
            out = this.newtonRoot(fcn, x0);
            disp("Correct ans = 5")
            fprintf("Result = %.3f\n", out);
        end
    end

    %% Private implementation
    methods (Access = private)
        function vMax = solveMaxSpeedInCorner(this, R)
            % Returns the maximum steady-state speed [m/s] for a given corner radius R [m]
            % using load-dependent lateral friction limits on front and rear axles.
            g = 9.81;
            m = this.mCarTotal;
            wb = this.wheelbase;
            a = this.rWeightBalF * wb;      % distance CoG -> front axle TODO: this should be (1 - this.rWeightBalF)
            b = wb - a;                      % distance CoG -> rear axle

            % Define residual functions for each axle being the limiting one
            resFront = @(v) this.corneringResidual(v, R, m, wb, b, 'front');
            resRear  = @(v) this.corneringResidual(v, R, m, wb, a, 'rear');

            % Initial guess using the old formula (constant peak mu, no load dependence)
            muPeak = this.tire.muTyreLat_peak;
            vGuess = sqrt( (muPeak * R * m * g) / (m - muPeak * R * this.aeroDownforceFactor) );
            % TODO: Remove this fallback (which I'm assuming is for max vCar without downforce for point mass).
            % If the vGuess has a complex value, it should error out, not take the real component and silently
            % continue
            if imag(vGuess) ~= 0 || ~isfinite(vGuess)
                vGuess = sqrt(R * muPeak * g);   % fallback
            end
            vGuess = max(real(vGuess), 0.1);

            % Solve for front-limited and rear-limited speeds using Newton-Raphson
            vFront = this.newtonRoot(resFront, vGuess);
            vRear  = this.newtonRoot(resRear,  vGuess);

            % The maximum achievable speed is the lower of the two
            vMax = min(vFront, vRear);
        end

        function r = corneringResidual(this, v, R, m, wb, dist, axle)
            % Residual for the equation: a_y_req - a_y_limit(axle) = 0
            % dist = distance from CoG to the *other* axle (a for rear limit, b for front limit)
            a_y_req = v^2 / R;

            % Compute vertical loads (longitudinal g=0)
            [Fz_axle, ~] = this.calculateAxleLoads(v, 0);
            if strcmp(axle, 'front')
                Fz = Fz_axle;   % front axle total load
            else
                [~, Fz] = this.calculateAxleLoads(v, 0);  % rear axle total load
            end

            % Per-tire load (assuming equal left/right distribution)
            % TODO: We are doing a roll rate sweep which changes load transfer distribution. We need to take into
            % account the load transfer for it to have any effect. Cannot assume same Fz left & right
            Fz_tire = Fz / 2;

            % Dynamic friction coefficient for lateral direction on this axle
            mu_dyn = this.tire.calculate_tyrefrictioncoefficient(Fz_tire, 'lat', axle);

            % Maximum lateral acceleration allowed by this axle (bicycle model)
            a_y_limit = (mu_dyn * Fz * wb) / (dist * m);

            r = a_y_req - a_y_limit;
        end

        function x = newtonRoot(this, fun, x0)
            % Simple Newton-Raphson solver with numerical derivative
            tol = 1e-6;
            maxIter = 30;
            x = x0;
            for iter = 1:maxIter
                fx = fun(x);
                if abs(fx) < tol
                    return;
                end
                % Numerical derivative
                h = max(1e-6 * x, 1e-8);
                df = (fun(x + h) - fun(x - h)) / (2*h);
                if abs(df) < 1e-12
                    break;
                end
                x_new = x - fx / df;
                if abs(x_new - x) < tol
                    x = x_new;
                    return;
                end
                x = x_new;
            end
            % Fallback to fzero if Newton fails
            x = fzero(fun, [0.01, x0*10]);
        end
    end
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


        function MMotor = calculate_MMotorFromFxTyreRear(this, FxTyreRear)
            MWheel = FxTyreRear .* this.tire.radiusTyreRollingRear;
            MMotor = MWheel ./ this.totalGearRatio;
            MMotor = MMotor ./ this.eTransmission;
        end


        function gLongTractionLimited = calculate_gLongTractionLimitedStraightLine(this, vCar, gLongPrev)
            [FzFront, FzRear] = this.calculateAxleLoads(vCar, gLongPrev);
            % Prevent wheelies
            FzFrontNegative = FzFront;
            FzFrontNegative(FzFrontNegative >= 0) = 0;

            % Dynamic friction coefficient
            [muDynamicRear, ~] = this.tire.calculate_tyrefrictioncoefficient(FzRear./2, 'long', 'rear');

            % FzFront = FzFront - FzFrontNegative;
            FzRear = FzRear + FzFrontNegative;
            gLongTractionLimited = FzRear .* muDynamicRear ./ this.mCarTotal;
        end


        function gLongPowerLimited = calculate_gLongPowerLimited(this, vCar)
            nWheelRear = vCar ./ this.tire.radiusTyreRollingRear;
            nMotor = nWheelRear .* this.totalGearRatio;
            MMotorMax = this.nMotorToMMotorMax(nMotor);
            if isnan(MMotorMax)
                % We are outside the motor torque map. We will assume that the last nMotor value in the map is the max
                % rpm of the motor and give 0 power here.
                MMotorMax = 0;
            end
            MWheelMax = MMotorMax .* this.totalGearRatio .* this.eTransmission;
            FxTyreRearPowerLimited = MWheelMax ./ this.tire.radiusTyreRollingRear;
            [~, ~, FDrag] = this.calculateAeroLoads(vCar);
            gLongPowerLimited = (FxTyreRearPowerLimited - FDrag) ./ this.mCarTotal;
        end


        function gLongBraking = calculate_gLongBraking(this, vCar, gLongNext)
            [FzFront, FzRear] = this.calculateAxleLoads(vCar, gLongNext);
            [muDynamicFront, ~] = this.tire.calculate_tyrefrictioncoefficient(FzFront./2, 'long', 'front');
            [muDynamicRear, ~]  = this.tire.calculate_tyrefrictioncoefficient(FzRear./2, 'long', 'rear');
            [~, ~, FDrag] = this.calculateAeroLoads(vCar);
            gLongBraking = (-(muDynamicFront.*FzFront + muDynamicRear.*FzRear) - FDrag) ./ this.mCarTotal;
        end
    end
end