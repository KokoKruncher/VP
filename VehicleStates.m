classdef VehicleStates < matlab.mixin.Copyable
    properties (Constant)
        loggedStates string = [
            "sRun";
            "tRun";
            "rCorner"
            "vCar";
            "gLong";
            "gLat";
            "gLongTractionLimited";
            "gLongPowerLimited";
            "MMotor";
            "FzFront";
            "FzRear"
            "FLiftF";
            "FLiftR";
            "FDrag";
            "LPUAF";
            "LPUAR";
            "muDynamicF";
            "muDynamicR"]
    end


    properties (SetAccess = private)
        results table
    end


    properties (Access = private)
        stepCount
        loggableStateKeys dictionary
    end

    %% Constructor
    methods
        function obj = VehicleStates(sRun)
            arguments
                sRun (:,1) double {mustBeFinite, mustBeNonnegative} = []
            end
            assert(issorted(sRun, "strictascend"), "sRun must be monotonically increasing with no duplicates.");
            loggableStates = obj.loggedStates;
            loggableStates(loggableStates == "sRun") = [];
            obj.loggableStateKeys = dictionary(loggableStates, true);
            obj.initialiseResultsTable(sRun);
        end
    end

    %% Public interface
    methods
        function log(this, param, value, index)
            arguments
                this (1,1) VehicleStates
                param (1,1) string
                value (:,1) double
                index (:,1) double
            end
            valueCount = numel(value);
            indexCount = numel(index);
            assert(valueCount == indexCount, "Number of logged values and number of indices must match.");
            assert(this.loggableStateKeys.isKey(param), ...
                "Paramater: '%s' is not a valid/loggable parameter. " + ...
                "Please add it to VehicleStates.loggedStates to make it loggable.", param);

            this.results.(param)(index) = value;
        end


        function logConstant(this, param, value)
            arguments
                this (1,1) VehicleStates
                param (1,1) string
                value (1,1) double
            end
            assert(this.loggableStateKeys.isKey(param), ...
                "Paramater: '%s' is not a valid/loggable parameter. " + ...
                "Please add it to VehicleStates.loggedStates to make it loggable.", param);
            
            this.results.(param)(:) = value;
        end


        function varargout = crop(this, args)
            arguments
                this (1,1) VehicleStates
                args.StartIndex double {mustBeInteger} = 1
                args.EndIndex double {mustBeScalarOrEmpty, mustBeInteger} = this.stepCount
            end
            assert(args.StartIndex >= 1, "Start index cannot be less than 1.");
            assert(args.StartIndex <= this.stepCount, "Start index cannot be more than the number of steps.");
            assert(args.EndIndex <= this.stepCount, "End index cannot be more than the number of steps.");
            assert(args.StartIndex <= args.EndIndex, "Start index cannot be more than the end index.");

            this.results = this.results(args.StartIndex:args.EndIndex, :);
            this.stepCount = height(this.results);
            if nargout > 0
                varargout{1} = this;
            end
        end


        function varargout = append(this, otherState)
            arguments
                this (1,1) VehicleStates
                otherState (1,1) VehicleStates
            end
            
            if ~isempty(this.results)
                % Make sure the states are continous and fix sRun mismatch if any.
                otherStateResults = otherState.results;
                firstDistanceStep = otherStateResults.sRun(2) - otherStateResults.sRun(1);
                % otherStateResults = otherStateResults(2:end, :);
                otherStateResults.sRun ...
                    = otherStateResults.sRun - otherStateResults.sRun(1) + this.results.sRun(end) + firstDistanceStep;

                this.results = [this.results; otherStateResults];
            else
                this.results = otherState.results;
            end
            % Recalculate tRun
            this.results.tRun = cumtrapz(this.results.sRun, 1 ./ this.results.vCar);
            this.stepCount = height(this.results);
            
            if nargout > 0
                varargout{1} = this;
            end
        end
        
        
        function bool = isEmpty(this)
            arguments
                this (1,1) VehicleStates
            end
            bool = isempty(this.result);
        end
        
        
        function varargout = debugPlots(this)
            hFig = figure(Name="Lap debug plots.");
            hAxes = axes(hFig);
            scatter(hAxes, this.results.sRun, this.results.vCar);
            grid(hAxes, "on");
            xlabel(hAxes, "sRun (m)");
            ylabel(hAxes, "vCar (m/s)");
            
            if nargout == 1
                varargout{1} = hFig;
            end
        end
    end

    %% Private implementation
    methods (Access = private)
        function initialiseResultsTable(this, sRun)
            this.stepCount = numel(sRun);
            variables = this.loggedStates;
            variableCount = numel(variables);
            tbl = array2table(nan(this.stepCount, variableCount), VariableNames=variables);
            tbl.sRun = sRun;

            this.results = tbl;
        end
    end
end