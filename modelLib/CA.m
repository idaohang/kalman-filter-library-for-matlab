classdef CA < Model
    %CA Constant Acceleration Model
    %   Constant Acceleration Model. For all further particulars
    %   see Estimation with Applications to Tracking and Navigation
    %   from Bar-Shalom
    properties (Constant)
        stateDescription=['s';'v';'a'];
    end
    
    properties
        T
        F
        Q
    end
    
    methods
        function obj = CA (params)
            obj.T=params;
            obj.F=getStateTransitionMatrix(obj, params);
        end
    end
    
    methods (Access = private)
        function F = getStateTransitionMatrix(obj, params)
            F=[1 params 0.5*params^2;0 1 params;0 0 1];
        end
        
        function Q = getProcessNoiseCovariance(obj, inModel, inVar)
            % DiscreteWiener Process Acceleration Model (DWPA)
            if strcmp(inModel,'DWPA')
                Q=[(1/4)*obj.T^4 (1/2)*obj.T^3 (1/2)*obj.T^2;(1/2)*obj.T^3 obj.T^2 obj.T;(1/2)*obj.T^2 obj.T 1]*inVar;
            end
            % ContinuousWiener Process Acceleration Model (CWPA)
            if strcmp(inModel,'CWPA')
                Q=[(1/20)*obj.T^5 (1/8)*obj.T^4 (1/6)*obj.T^3;(1/8)*obj.T^4 (1/3)*obj.T^3 (1/2)*obj.T^2;(1/6)*obj.T^3 (1/2)*obj.T^2 obj.T]*inVar;
            end
        end
    end
end

