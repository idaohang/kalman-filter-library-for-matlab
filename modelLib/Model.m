classdef (Abstract) Model < handle
    
    properties (Constant, Abstract)
        
        stateDescription;
        %shortDescription;
        %paramsDefault;
    end
    
    properties (Abstract)
        T
        stateTransitionMatrix
    end
    
    methods (Abstract)
        %init (obj, input);
        %predict (obj, input);
        %update (obj, input);
        %state = getOrientation (obj);
    end
    
end

