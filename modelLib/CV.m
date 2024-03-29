classdef CV < Model
    %CV Constant Velocity Model
    %   Constant Velocity Model. For all further particulars
    %   see Estimation with Applications to Tracking and Navigation
    %   from Bar-Shalom
    properties (Constant)
        stateDescription=['s';'v'];
    end
    
    properties
        T
        F
        Q
        Gamma
        
        stateConfiguration
    end
    
    methods
        function obj = CV (params)
            obj.T=params;
            obj.F=getStateTransitionMatrix(obj, params);
            % default process noise model is DWNA (more common)
            obj.Q=getProcessNoiseCovariance(obj, 'DWNA', 1);
            obj.Gamma=getProcessNoiseGain(obj, 'DWNA');
        end
    end
    methods
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
            F=[1 params;0 1];
        end
        function Q = getProcessNoiseCovariance(obj, inModel, inVar)
            % Discrete White Noise Acceleration Model (DWNA)
            if strcmp(inModel,'DWNA')
                Q=[(1/4)*obj.T^4 (1/2)*obj.T^3;(1/2)*obj.T^3 obj.T^2]*inVar;
            end
            % Continuous White Noise Acceleration Model (CWNA)
            if strcmp(inModel,'CWNA')
                Q=[(1/3)*obj.T^3 (1/2)*obj.T^2;(1/2)*obj.T^2 obj.T]*inVar;
            end
        end
        function Gamma = getProcessNoiseGain(obj, inModel)
            % Discrete White Noise Acceleration Model (DWNA)
            if strcmp(inModel,'DWNA')
                Gamma=[0.5*obj.T^2;obj.T];
            end
            % Continuous White Noise Acceleration Model (CWNA)
            if strcmp(inModel,'CWNA')
                Gamma=[0;1];
            end
        end
    end
end