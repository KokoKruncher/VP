classdef Vehicle < handle
    % A vehicle that can perform simple steady-state manouvres.
    properties
        mCarTotal               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rWeightBalF             double {mustBeScalarOrEmpty, mustBeFinite, mustBeInRange(rWeightBalF, 0, 1)}
        wheelbase               double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        trackWidthF             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        trackWidthR             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        hCoG                    double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        hRollCentreF            double {mustBeScalarOrEmpty, mustBeFinite}
        hRollCentreR            double {mustBeScalarOrEmpty, mustBeFinite}
        kSpringF                double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}              
        kSpringR                double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive} 
        motionRatioF            double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive} % Wheel/spring
        motionRatioR            double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive} % Wheel/spring
        kWheelARBF              double {mustBeScalarOrEmpty, mustBeFinite, mustBeNonnegative} 
        kWheelARBR              double {mustBeScalarOrEmpty, mustBeFinite, mustBeNonnegative} 
        rAeroBalF               double {mustBeScalarOrEmpty, mustBeFinite, mustBeInRange(rAeroBalF, 0, 1)}
        aeroDragFactor          double {mustBeScalarOrEmpty, mustBeFinite, mustBeNonnegative}
        aeroDownforceFactor     double {mustBeScalarOrEmpty, mustBeFinite, mustBeNonnegative}
        rFinalDrive             double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        rTransmissionRatio      double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
        eTransmission           double {mustBeScalarOrEmpty, mustBeFinite, mustBeInRange(eTransmission, 0.01, 1)}
        nMotorMapLookup         (:,1) double {mustBeFinite, mustBeNonnegative} % radians per second
        MMotorMapLookup         (:,1) double {mustBeFinite, mustBeNonnegative}
        tireF                   LoadDependentTireModel
        tireR                   LoadDependentTireModel
    end

    % Pre-calculated values
    properties (SetAccess = private)
        weight double {mustBeScalarOrEmpty}
        FzFrontStatic double {mustBeScalarOrEmpty}
        FzRearStatic double {mustBeScalarOrEmpty}
        nMotorToMMotorMax {mustBeA(nMotorToMMotorMax, 'griddedInterpolant')} = griddedInterpolant()
        totalGearRatio double {mustBeScalarOrEmpty}
        kWheelCornerSpringF double {mustBeScalarOrEmpty}
        kWheelCornerSpringR double {mustBeScalarOrEmpty}
        cogDistanceToFrontAxle double {mustBeScalarOrEmpty}
        cogRollMomentArm double {mustBeScalarOrEmpty}
        hRollAxisAtCoG double {mustBeScalarOrEmpty}
        kRollElasticF double {mustBeScalarOrEmpty}
        kRollElasticR double {mustBeScalarOrEmpty}
        kRollElasticTotal double {mustBeScalarOrEmpty}
        kRollTyreF double {mustBeScalarOrEmpty}
        kRollTyreR double {mustBeScalarOrEmpty}
        kRollTyreTotal double {mustBeScalarOrEmpty}
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
            this.kWheelCornerSpringF = this.kSpringF ./ (this.motionRatioF .^ 2);
            this.kWheelCornerSpringR = this.kSpringR ./ (this.motionRatioR .^ 2);
            this.cogDistanceToFrontAxle = (1 - this.rWeightBalF) .* this.wheelbase;
            
            this.hRollAxisAtCoG = ((this.hRollCentreR - this.hRollCentreF) ./ this.wheelbase) ...
                .* this.cogDistanceToFrontAxle + this.hRollCentreF;
            this.cogRollMomentArm = this.hCoG - this.hRollAxisAtCoG;
            
            % Aggregate wheel rate in roll (N/m)
            kWheelRollElasticF = this.calculateSeriesSpringRate(this.kWheelCornerSpringF + this.kWheelARBF, this.tireF.kSpring);
            kWheelRollElasticR = this.calculateSeriesSpringRate(this.kWheelCornerSpringR + this.kWheelARBR, this.tireR.kSpring);
            
            % Aggregate roll rate (Nm/rad)
            this.kRollElasticF = kWheelRollElasticF .* (this.trackWidthF .^ 2) ./ 2;
            this.kRollElasticR = kWheelRollElasticR .* (this.trackWidthR .^ 2) ./ 2;
            this.kRollElasticTotal = this.kRollElasticF + this.kRollElasticR;
            
            % Tyre contribution to roll rate (Nm/rad)
            this.kRollTyreF = this.tireF.kSpring .* (this.trackWidthF .^ 2) ./ 2;
            this.kRollTyreR = this.tireR.kSpring .* (this.trackWidthR .^ 2) ./ 2;
            this.kRollTyreTotal = this.kRollTyreF + this.kRollTyreR;
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
            if this.tireF.decayCoeff == 0 && this.tireR.decayCoeff == 0
                % Constant mu, so can use more basic equation.
                % Calculate separate vCar limit for front and rear axle, then take the minimum of the two.
                massFront = this.mCarTotal .* this.rWeightBalF;
                massRear = this.mCarTotal .* (1 - this.rWeightBalF);
                downforceFactorFront = this.aeroDownforceFactor .* this.rAeroBalF;
                downforceFactorRear = this.aeroDownforceFactor .* (1 - this.rAeroBalF);
                muFront = this.tireF.muTyreLat_peak;
                muRear = this.tireR.muTyreLat_peak;
                
                vCarLimitFront ...
                    = this.calculateConstantMuAxleCornerSpeedLimit(muFront, massFront, downforceFactorFront, cornerRadius);
                vCarLimitRear ...
                    = this.calculateConstantMuAxleCornerSpeedLimit(muRear, massRear, downforceFactorRear, cornerRadius);
                
                vCar = min(vCarLimitFront, vCarLimitRear);
            else
                % Load dependent tyre requires iterative approach
                vCar = this.solveMaxSpeedInCorner(cornerRadius);
            end
            gLat = this.calculate_gLat(vCar, cornerRadius);
            
            state = VehicleStates(sRun);
            state.logConstant("rCorner", cornerRadius);
            state.logConstant("vCar", vCar);
            state.logConstant("gLong", gLong);
            state.logConstant("gLat", gLat);
            this.backCalculateStates(state);

            % Log dynamic friction coefficients (per axle, using per‑tire loads)
            [muDynamicFront, LPUAF] = this.tireF.calculate_tyrefrictioncoefficient(...
                state.results.FzFront/2, 'lat');
            [muDynamicRear, LPUAR]  = this.tireR.calculate_tyrefrictioncoefficient(...
                state.results.FzRear/2, 'lat');
            indices = 1:numel(sRun);
            state.log("LPUAF", LPUAF, indices);
            state.log("muDynamicF", muDynamicFront, indices);
            state.log("LPUAR", LPUAR, indices);
            state.log("muDynamicR", muDynamicRear, indices);
        end
        
        
        function state = driveSteadyStateCornerAtSpeed(this, sRun, cornerRadius, cornerSpeed)
            arguments
                this Vehicle
                sRun
                cornerRadius (1,1) double
                cornerSpeed (1,1) double
            end
            gLong = 0;
            vCar = cornerSpeed;
            gLat = this.calculate_gLat(vCar, cornerRadius);
            
            state = VehicleStates(sRun);
            state.logConstant("rCorner", cornerRadius);
            state.logConstant("vCar", vCar);
            state.logConstant("gLong", gLong);
            state.logConstant("gLat", gLat);
            this.backCalculateStates(state);

            % Log dynamic friction coefficients (per axle, using per‑tire loads)
            [muDynamicFront, LPUAF] = this.tireF.calculate_tyrefrictioncoefficient(...
                state.results.FzFront/2, 'lat');
            [muDynamicRear, LPUAR]  = this.tireR.calculate_tyrefrictioncoefficient(...
                state.results.FzRear/2, 'lat');
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

            [muDynamicFront, LPUAF] = this.tireF.calculate_tyrefrictioncoefficient(state.results.FzFront./2, 'long');
            [muDynamicRear, LPUAR] = this.tireR.calculate_tyrefrictioncoefficient(state.results.FzRear./2, 'long');
            state.log("LPUAF", LPUAF, indices);
            state.log("muDynamicF", muDynamicFront, indices);
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

            [muDynamicFront, LPUAF] = this.tireF.calculate_tyrefrictioncoefficient(state.results.FzFront./2, 'long');
            [muDynamicRear, LPUAR]  = this.tireR.calculate_tyrefrictioncoefficient(state.results.FzRear./2, 'long');
            state.log("LPUAF", LPUAF, indices);
            state.log("muDynamicF", muDynamicFront, indices);
            state.log("LPUAR", LPUAR, indices);
            state.log("muDynamicR", muDynamicRear, indices);
        end
    end

    %% Private implementation
    methods (Access = private)
        function vMax = solveMaxSpeedInCorner(this, R)
            % Returns the maximum steady-state speed [m/s] for a given corner radius R [m]
            % using load-dependent lateral friction limits on front and rear axles.
            g = 9.81;
            m = this.mCarTotal;
            % wb = this.wheelbase;
            % a = (1 - this.rWeightBalF) * wb;      % distance CoG -> front axle
            % b = wb - a;                      % distance CoG -> rear axle

            % Define residual functions for each axle being the limiting one
            resFront = @(v) this.corneringResidual(v, R,'front');
            resRear  = @(v) this.corneringResidual(v, R,'rear');
            
            % Initial guess using the simplified formula (point mass, constant peak mu, no load dependence)
            muPeakAvg = (this.tireF.muTyreLat_peak + this.tireR.muTyreLat_peak) ./ 2;
            vGuess = sqrt( (muPeakAvg * R * m * g) / (m - muPeakAvg * R * this.aeroDownforceFactor) );

            % Solve for front-limited and rear-limited speeds using Newton-Raphson
            vFront = this.newtonRoot(resFront, vGuess);
            vRear  = this.newtonRoot(resRear,  vGuess);

            % The maximum achievable speed is the lower of the two
            vMax = min(vFront, vRear);
        end

        
        function a_y_residual = corneringResidual(this, vCar_guess, corner_radius, axle)
            % Residual for the equation: a_y_req - a_y_limit(axle) = 0
            % dist = distance from CoG to the *other* axle (a for rear limit, b for front limit)
            a_y_req = vCar_guess.^2 ./ corner_radius;

            % Compute vertical loads (longitudinal g=0)
            [FzFront, FzRear] = this.calculateAxleLoads(vCar_guess, 0);
            [loadTransferF, loadTransferR] = this.calculateLoadTransfer(a_y_req);
            if strcmp(axle, 'front')
                Fz_axle = FzFront;   % front axle total load
                axle_mass = this.mCarTotal .* this.rWeightBalF;
                delta_Fz = loadTransferF;
                tire = this.tireF;
            elseif strcmp(axle, 'rear')
                Fz_axle = FzRear;  % rear axle total load
                axle_mass = this.mCarTotal .* (1 - this.rWeightBalF);
                delta_Fz = loadTransferR;
                tire = this.tireR;
            else
                error("Invalid axle: %s", axle);
            end

            %  Calculate outer and inner tire loads
            % Using max(..., 0) to prevent negative loads (inner tire lifting off)
            Fz_outer = max((Fz_axle ./ 2) + delta_Fz, 0);
            Fz_inner = max((Fz_axle ./ 2) - delta_Fz, 0);

            %  Calculate dynamic mu for each tire based on its specific load
            mu_outer = tire.calculate_tyrefrictioncoefficient(Fz_outer, 'lat');
            mu_inner = tire.calculate_tyrefrictioncoefficient(Fz_inner, 'lat');
            
            %  Total maximum lateral force capable by this axle
            Fy_max_axle = (mu_outer .* Fz_outer) + (mu_inner .* Fz_inner);

            % Maximum vehicle lateral acceleration allowed by this axle limit
            a_y_limit = Fy_max_axle ./ axle_mass;

            a_y_residual = a_y_req - a_y_limit;
        end
        
        
        function [loadTransferF, loadTransferR, totalRollAngle] = calculateLoadTransfer(this, gLat)
            g = 9.81;
            elasticRollAngle = (this.mCarTotal .* gLat .* this.cogRollMomentArm) ...
                ./ (this.kRollElasticTotal - (this.mCarTotal .* g .* this.cogRollMomentArm));
            inelasticRollAngle =  (this.mCarTotal .* gLat .* this.hRollAxisAtCoG) ./ this.kRollTyreTotal;
            totalRollAngle = elasticRollAngle + inelasticRollAngle;
            
            loadTransferF = (this.kRollElasticF .* elasticRollAngle + this.kRollTyreF .* inelasticRollAngle) ./ ...
                this.trackWidthF;
            loadTransferR = (this.kRollElasticR .* elasticRollAngle + this.kRollTyreR .* inelasticRollAngle) ./ ...
                this.trackWidthR;
        end
    end
    
    
    methods (Access = private, Static)
        function tRun = calculate_tRun(sRun, vCar)
            tRun = cumtrapz(sRun, 1 ./ vCar);
        end


        function gLat = calculate_gLat(vCar, radiusCorner)
            gLat = (vCar .^ 2) ./ radiusCorner;
        end
        
        
        function vCarLimit = calculateConstantMuAxleCornerSpeedLimit(muAxle, axleMass, axleDownforceFactor, cornerRadius)
            g = 9.81;
            vCarLimit = sqrt( (muAxle .* cornerRadius .* axleMass .* g) ...
                    ./ (axleMass - muAxle .* cornerRadius .* axleDownforceFactor) );
        end
        
        
        function k = calculateSeriesSpringRate(k1, k2)
            kInv = (1 ./ k1) + (1 ./ k2);
            k = 1 ./ kInv;
        end
        
        
        function x = newtonRoot(fun, x0)
            % Simple Newton-Raphson solver with numerical derivative
            % fun: function handle for the objective function to solve, fun(x) = 0
            % x0 : Initial guess for for the solution to the objective function.
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
            % x = fzero(fun, [0.01, x0*10]);
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
            FxTyreRearTraction = this.mCarTotal .* state.results.gLong + FDrag;
            MMotor = this.calculate_MMotorFromFxTyreRear(FxTyreRearTraction);

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
            MWheel = FxTyreRear .* this.tireR.rollingRadius;
            MMotor = MWheel ./ this.totalGearRatio;
            MMotor = MMotor ./ this.eTransmission;
        end


        function gLongTractionLimited = calculate_gLongTractionLimitedStraightLine(this, vCar, gLongPrev)
            [FzFront, FzRear] = this.calculateAxleLoads(vCar, gLongPrev);
            % Prevent wheelies
            FzFrontNegative = FzFront;
            FzFrontNegative(FzFrontNegative >= 0) = 0;

            % Dynamic friction coefficient
            [muDynamicRear, ~] = this.tireR.calculate_tyrefrictioncoefficient(FzRear./2, 'long');

            % FzFront = FzFront - FzFrontNegative;
            FzRear = FzRear + FzFrontNegative;
            [~, ~, FDrag] = this.calculateAeroLoads(vCar);
            gLongTractionLimited = (FzRear .* muDynamicRear - FDrag) ./ this.mCarTotal;
        end


        function gLongPowerLimited = calculate_gLongPowerLimited(this, vCar)
            nWheelRear = vCar ./ this.tireR.rollingRadius;
            nMotor = nWheelRear .* this.totalGearRatio;
            MMotorMax = this.nMotorToMMotorMax(nMotor);
            if isnan(MMotorMax)
                % We are outside the motor torque map. We will assume that the last nMotor value in the map is the max
                % rpm of the motor and give 0 power here.
                MMotorMax = 0;
            end
            MWheelMax = MMotorMax .* this.totalGearRatio .* this.eTransmission;
            FxTyreRearPowerLimited = MWheelMax ./ this.tireR.rollingRadius;
            [~, ~, FDrag] = this.calculateAeroLoads(vCar);
            gLongPowerLimited = (FxTyreRearPowerLimited - FDrag) ./ this.mCarTotal;
        end


        function gLongBraking = calculate_gLongBraking(this, vCar, gLongNext)
            [FzFront, FzRear] = this.calculateAxleLoads(vCar, gLongNext);
            [muDynamicFront, ~] = this.tireF.calculate_tyrefrictioncoefficient(FzFront./2, 'long');
            [muDynamicRear, ~]  = this.tireR.calculate_tyrefrictioncoefficient(FzRear./2, 'long');
            [~, ~, FDrag] = this.calculateAeroLoads(vCar);
            gLongBraking = (-(muDynamicFront.*FzFront + muDynamicRear.*FzRear) - FDrag) ./ this.mCarTotal;
        end
    end
end