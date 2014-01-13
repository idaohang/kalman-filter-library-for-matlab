classdef (Abstract) Model < handle
    
    properties (Constant, Abstract)
        stateDescription;
        %shortDescription;
        %paramsDefault;
    end
    
    properties (Abstract)
        T % sample rate
        F % state transition matrix
        Q % process noise covariance matrix
        var
    end
    
    methods (Abstract)
        %init (obj, input);
        %predict (obj, input);
        %update (obj, input);
        %state = getOrientation (obj);
    end
    
end

