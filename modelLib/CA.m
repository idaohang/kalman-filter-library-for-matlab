classdef CA < Model
    %CA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        T
        stateTransitionMatrix
    end
    
    properties (Constant)
        stateDescription=['s';'v';'a'];
        % x=[s;v]
        % x=[s;s;s;]
        % ('s','s_x','s','s_y','s','s_z','v','v_x','v','v_y','v','v_z')
    end
    
    methods
        function obj = CA (params)
            obj.T=params;
            obj.stateTransitionMatrix=getStateTransitionMatrix(obj, params);
        end
    end
    
    methods (Access = private)
        function F = getStateTransitionMatrix(obj, params)
            F=[1 params 0.5*params^2;0 1 params;0 0 1];
        end
    end
end

