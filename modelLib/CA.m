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
        Gamma
        var
        
        stateConfiguration
    end
    
    methods
        function obj = CA (params,inVar)
            obj.T=params;
            obj.var=inVar;
            obj.F=getStateTransitionMatrix(obj, params);
            % default process noise model is DWPA (more common)
            obj.Q=getProcessNoiseCovariance(obj, 'DWPA');
            obj.Gamma=getProcessNoiseGain(obj, 'DWPA');
            obj.stateConfiguration={obj.stateDescription(1),obj.stateDescription(2),obj.stateDescription(3)};
        end
        
        function setStateTransitionMatrix(obj, varargin)
            [obj.F,obj.stateConfiguration]=stateTransitionMatrixBuilder(obj.F,obj.stateDescription,varargin);
            obj.Gamma=processNoiseVectorBuilder(obj.Gamma,obj.stateDescription,varargin);
            obj.Q=obj.Gamma*obj.Gamma';
            %todo set procesNoiseMatrix
        end
        
        function setProcessNoiseGain(obj, inGamma)
            obj.Gamma=inGamma;
        end
    end
    
    methods (Access = private)
        function F = getStateTransitionMatrix(obj, params)
            F=[1 params 0.5*params^2;0 1 params;0 0 1];
        end
        
        function Q = getProcessNoiseCovariance(obj, inModel)
            % DiscreteWiener Process Acceleration Model (DWPA)
            if strcmp(inModel,'DWPA')
                Q=[(1/4)*obj.T^4 (1/2)*obj.T^3 (1/2)*obj.T^2;(1/2)*obj.T^3 obj.T^2 obj.T;(1/2)*obj.T^2 obj.T 1]*obj.var;
            end
            % ContinuousWiener Process Acceleration Model (CWPA)
            if strcmp(inModel,'CWPA')
                Q=[(1/20)*obj.T^5 (1/8)*obj.T^4 (1/6)*obj.T^3;(1/8)*obj.T^4 (1/3)*obj.T^3 (1/2)*obj.T^2;(1/6)*obj.T^3 (1/2)*obj.T^2 obj.T]*obj.var;
            end
        end
        
        function Gamma = getProcessNoiseGain(obj, inModel)
            % Discrete White Noise Acceleration Model (DWNA)
            if strcmp(inModel,'DWPA')
                Gamma=[0.5*obj.T^2;obj.T;1];
            end
            % ContinuousWiener Process Acceleration Model (CWPA)
            if strcmp(inModel,'CWPA')
                Gamma=[0;0;1];
            end
        end
    end
end

