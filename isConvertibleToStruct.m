classdef (Abstract) isConvertibleToStruct < handle
    methods
        function S = toStruct(this)
            arguments
                this (1,1)
            end
            params = metaclass(this).PropertyList;
            nParams = numel(params);

            S = struct();
            for ii = 1:nParams
                if strcmp(params(ii).SetAccess, 'private')
                    continue
                end

                thisParamName = params(ii).Name;
                thisParam = this.(thisParamName);
                if ~isa(thisParam, "handle")
                    S.(thisParamName) = thisParam;
                elseif isa(thisParam, "isConvertibleToStruct")
                    S.(thisParamName) = thisParam.toStruct();
                end
            end
        end
    end
end