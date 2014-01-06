classdef KalmanFilter<handle
    
    properties (SetAccess = public)
        T;      % sample period
        F;      % system matrix
        H;      % messurement matrix
        Q;      % covariance matrix process noise
        R;      % covariance matrix messurement noise
        
        G;      % input gain matrix
        
        x;      % state vector
        P;      % covariance matrix
    end
    properties (SetAccess = private)
        S;      % innovation matrix
        K;      % kalman gain matrix
        
        state;
        covariance;
    end
    
    methods
        %% Constructor
        function obj = KalmanFilter(inModel)
            obj.T = inModel.T;
            obj.F = inModel.F;
            obj.Q = inModel.Q;
        end
        
        %% INITIALIZATION
        function initInitialState(obj, inX) % start value of state for recursion
            obj.x=inX;
        end
        function initInitialCovariance(obj, inP) % start value of covariance for recursion
            obj.P=inP;
        end
        
        %% RUN
        function obj = run(obj, measurements, u)
            if obj.checkValues()
                obj.state{1}=obj.x;
                obj.covariance{1}=obj.P;
                for i=2:length(measurements)
                    % Prediction
                    obj.x=obj.F*obj.x+obj.G*u(i);
                    obj.P=obj.F*obj.P*obj.F'+obj.Q;
                    z=obj.H*obj.x;
                    % Filering
                    obj.S=obj.R+obj.H*obj.P*obj.H';
                    obj.K=obj.P*obj.H'*pinv(obj.S);
                    obj.x=obj.x+obj.K*(measurements(i)-z);
                    %obj.P=obj.P-obj.K*obj.S*obj.K';
                    % Joseph Form
                    obj.P=(eye(size(obj.P,2))-obj.K*obj.H)*obj.P*(eye(size(obj.P,2))-obj.K*obj.H)'+obj.K*obj.R*obj.K';
                    % Save KF Result
                    obj.state{i}=obj.x;
                    if eig(obj.P)<0
                        display('Warning: Covariance is not positive definite');
                        obj.P
                    else
                        obj.covariance{i}=obj.P;
                    end
                end
                
            end
        end
        %% checkValues
        function [ret]=checkValues(obj)
            ret=1;
            if ~isvector(obj.x)
                ret=0;
                display('Error: x is not a Vector');
                obj.x
            end
        end
    end
end
