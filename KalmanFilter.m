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
        z       % predicted measurement
        
        measurments
        model;
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
            obj.model=inModel;
            obj.T = inModel.T;
            obj.F = inModel.F;
            obj.Q = inModel.Q;
            obj.G=0;
        end
        
        %% INITIALIZATION
        function initInitialState(obj, inX) % start value of state for recursion
            obj.x=inX;
        end
        function initInitialCovariance(obj, inP) % start value of covariance for recursion
            obj.P=inP;
        end
        
        %% CONFIGURATION MEASUREMENT MODEL
        function setMeasurmentModel(obj, varargin)
            obj.H=zeros(length(varargin),size(obj.F));
            for j=1:length(varargin)
                for i=1:length(obj.F)
                    if strcmp(obj.model.stateConfiguration{i},varargin{j})
                        obj.H(j,i)=1;
                    end
                end
            end
        end
        
        function setMeasurmentCovariance(obj, inR)
            obj.R=inR;
        end
        function predict(obj,x,u)
            obj.x=obj.F*x+obj.G*u;
            obj.P=obj.F*obj.P*obj.F'+obj.Q;
            obj.z=obj.H*obj.x;
        end
        function update(obj,measurements)
            obj.S=obj.R+obj.H*obj.P*obj.H';
            obj.K=obj.P*obj.H'*pinv(obj.S);
            obj.x=obj.x+obj.K*(measurements-obj.z);
            obj.P=(eye(size(obj.P,2))-obj.K*obj.H)*obj.P*(eye(size(obj.P,2))-obj.K*obj.H)'+obj.K*obj.R*obj.K';
        end
        %% RUN
        function obj = run(obj, measurements, varargin)
            if obj.checkValues()
                obj.state{1}=obj.x;
                obj.covariance{1}=obj.P;
                if length(varargin)
                    u=zeros(length(measurements));
                end
                for i=2:length(measurements)
                    predict(obj.x,u(i));
                    update(measurement(i));
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
