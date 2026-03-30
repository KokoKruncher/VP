classdef (Abstract) Tyre < handle
    properties
        rollingRadius double {mustBeScalarOrEmpty, mustBeFinite, mustBePositive}
    end
    
    
    methods (Abstract)
        calculateFxMax(this)
        calculateFyMax(this)
    end
end