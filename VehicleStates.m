classdef VehicleStates < handle
    properties (Constant)
        loggedStates string = [
            "sRun";
            "tRun";
            "vCar";
            "gLong";
            "gLat";
            "gLongTractionLimited";
            "gLongPowerLimited";
            "MMotor";
            "FzRear"
            "FLift";
            "FDrag"]
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
                sRun (:,1) double {mustBeNonempty, mustBeFinite, mustBeNonnegative}
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
                this 
                param (1,1) string 
                value double
                index double
            end
            valueCount = numel(value);
            indexCount = numel(index);
            assert(valueCount == indexCount, "Number of logged values and number of indices must match.");
            assert(this.loggableStateKeys.isKey(param), "Paramater: '%s' is not a valid/loggable parameter.", param);
            this.results.(param)(index) = value;
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