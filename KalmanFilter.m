classdef KalmanFilter<handle
    % Version 1.1 - www.kalman-filter.de - www.kalman-filter.com
    %KALMANFILTER general implementation of kalman filter
    %   Usage:
    %   1. Indicate system matrix F
    %   2. Indicate messurement matrix H
    %   3. Indicate sample period T
    %   4. Indicate covariance matrix process noise Q
    %   5. Indicate covariance matrix messurement noise R
    %   6. Indicate gain matrix G
    %   7. Indicate state and covariance with method InitState &
    %   InitCovariance
    
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
        
        % INITIALIZATION
        function initInitialState(obj, inX) % start value of state for recursion
            obj.x=inX;
        end
        function initInitialCovariance(obj, inP) % start value of covariance for recursion
            obj.P=inP;
        end
        %%
        function [ret]=reset(obj)
            obj.P=P_init;
            ret=1;
        end
        
        % RUN
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
        %% getState(line)
        %  For line=i with i=1, ... return value is state element at position i
        %  For line<1 return value is state vector as an array
        function [x_line]=getState(obj,line)
            if line>0
                x_line=nan(length(obj.state),1);
                for i=1:length(obj.state)
                    x_line(i)=obj.state{i}(line);
                end
            else
                x_line=nan(size(obj.state{1},1),length(obj.state));
                for i=1:length(obj.state)
                    x_line(:,i)=obj.state{i};
                end
            end
        end
        %% getCovariance(line,column)
        function [P_ret]=getCovariance(obj, line, column)
            if line>0 && column>0
                P_ret=nan(1,length(obj.state));
                for i=1:length(obj.state)
                    P_ret(i)=obj.covariance{i}(line,column);
                end
            else
                for i=1:length(obj.state)
                    P_ret{i}=obj.covariance{i};
                end
            end
        end
    end
    methods (Access=private)
        function [x,P]=predict(obj)
            obj.x=obj.F*obj.x+obj.G*u(i);
            obj.P=obj.F*obj.P*obj.F'+obj.Q;
            z=obj.H*obj.x;
            % Filering
            %obj.S=obj.R+obj.H*obj.P*obj.H';
            %obj.K=obj.P*obj.H'*pinv(obj.S);
            %obj.x=obj.x+obj.K*(measurements(i)-z);
            %obj.P=obj.P-obj.K*obj.S*obj.K';
            % Joseph Form
            %obj.P=(eye(size(obj.P,2))-obj.K*obj.H)*obj.P*(eye(size(obj.P,2))-obj.K*obj.H)'+obj.K*obj.R*obj.K';
        end
    end
end

