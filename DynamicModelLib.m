classdef DynamicModelLib
    %DYNAMICMODELLIB Libary of Dynamic Models for Kalman Filtering
    %
    % CLASS DYNAMICMODELLIB
    %   DYNAMICMODELLIB(MODEL,DIMENSIONS,T) creates a class for dynamic
    %   Model MODELL with dimension DIMENSIONS and sample time T. See
    %   METHODS for How to use with a Kalman Filter.
    %
    %   MODEL can take the following values:
    %    2-degree(Position,Velocity):
    %    'DWNA' - Discrete White Noise Acceleration Model (Bar-Shalom)
    %    'NVEL' - Noisy Velocity private implementation
    %    'DVEL' - Decreasing Velocity private implementation
    %    3-degree(Position,Velocity,Acceleration):
    %    'NACC' - Noisy Acceleration private implementation
    %
    %   DIMENSIONS can take the following values:
    %    1 - For one-dimensional System (State e.g. [Position;Velocity])
    %    2 - For two-dimensional System (State e.g. [PosX;PosY;VelX;VelY])
    %    3 - For three-dimensional System (State analogous like 2)
    %
    % METHODS
    %   F=getStateTransitionMatrix(), returns the State Transistion Matrix
    %
    %
    %   Q=getProcessNoiseMatrix(), returns the Covariance Matrix for the
    %   Process Noise
    %
    % More Infos: www.kalman-filter.de - March 2013
    
    properties
        Model
        Dimensions
        T
    end
    
    methods
        function obj = DynamicModelLib(model,dimensions,t)
            obj.Model=model;
            if ~strcmp(model,'DWNA') && ~strcmp(model,'NVEL') && ~strcmp(model,'NACC') && ~strcmp(model,'DVEL') && ~strcmp(model,'IOSB') && ~strcmp(model,'BVEL');
                display(['Error:' model 'is unknown. Type help DynamicModelLib for Info.']);
            end
            
            obj.Dimensions=dimensions;
            if obj.Dimensions>3
                display('Error: Only Dimensions: 1, 2 or 3 are allowed');
            end
            
            obj.T=t;
        end
        function [n] = getStateDegree(obj)
            if strcmp(obj.Model,'DWNA') || strcmp(obj.Model,'NVEL') || strcmp(obj.Model,'DVEL') || strcmp(obj.Model,'IOSB')
                n=2;
            end
            if strcmp(obj.Model,'DWPA') || strcmp(obj.Model,'NACC') || strcmp(obj.Model,'BVEL')
                n=3;
            end
            if obj.Dimensions==2
                n=n*2;
            end
            if obj.Dimensions==3
                n=n*3;
            end
        end
        function [F]=getStateTransitionMatrix(obj,alpha)
            % 2 degree
            if strcmp(obj.Model,'DWNA') || strcmp(obj.Model,'NVEL')
                F=[1 obj.T;0 1];
                if obj.Dimensions==2
                    F=[F(1,1) 0 F(1,2) 0;0 F(1,1) 0 F(1,2);F(2,1) 0 F(2,2) 0; 0 F(2,1) 0 F(2,2)];
                end
                if obj.Dimensions==3
                    F=1;
                end
            end
            if strcmp(obj.Model,'IOSB')
                F=[1 obj.T;0 1];
                if obj.Dimensions==2
                    F=[F(1,1) 0 F(1,2) 0;0 F(1,1) 0 F(1,2);F(2,1) 0 F(2,2) 0; 0 F(2,1) 0 F(2,2)];
                end
                if obj.Dimensions==3
                    F=1;
                end
            end
            if strcmp(obj.Model,'DVEL')
                F=[1 obj.T;0 alpha(1)];
                if obj.Dimensions==2
                    F=[F(1,1) 0 F(1,2) 0;0 F(1,1) 0 F(1,2);F(2,1) 0 alpha(1) 0; 0 F(2,1) 0 alpha(2)];
                end
                if obj.Dimensions==3
                    F=1;
                end
            end
            % 3 degree
            if strcmp(obj.Model,'DWPA') || strcmp(obj.Model,'NACC')
                F=[1 obj.T 0.5*obj.T^2;0 1 obj.T; 0 0 1];
                if obj.Dimensions==2
                    F=[F(1,1) 0 F(1,2) 0 F(1,3) 0;0 F(1,1) 0 F(1,2) 0 F(1,3);F(2,1) 0 F(2,2) 0 F(2,3) 0; 0 F(2,1) 0 F(2,2) 0 F(2,3); F(3,1) 0 F(3,2) 0 F(3,3) 0; 0 F(3,1) 0 F(3,2) 0 F(3,3)];
                end
                if obj.Dimensions==3
                    F=[F(1,1) 0 0 F(1,2) 0 0;0 F(1,1) 0 0 F(1,2) 0;0 0 F(1,1) 0 0 F(1,2); F(2,1) 0 0 F(2,2) 0 0; 0 F(2,1) 0 0 F(2,2) 0; 0 0 F(2,1) 0 0 F(2,2)];
                end
            end
            %
            if strcmp(obj.Model,'BVEL')
                F=[1 0 obj.T 0 0 0;
                    0 1 0 obj.T 0 0;
                    0 0 1 0 0 0;
                    0 0 0 1 0 0;
                    0 0 0 0 1 obj.T;
                    0 0 0 0 0 1];
            end
        end
        function [G]=getInputGain(obj)
            if strcmp(obj.Model,'BVEL')
                G=[0;0;0;0;0.5*obj.T;obj.T];
            else
                G=0;
            end

        end
        function [Q]=getProcessNoiseMatrix(obj, var)
            if strcmp(obj.Model,'BVEL')
                Q=[0 0 0 0 0 0;
                    0 0 0 0 0 0;
                    0 0 var(1) 0 0 0;
                    0 0 0 var(2) 0 0;
                    0 0 0 0 0 0;
                    0 0 0 0 0 0];
            end
            % 2 degree
            if strcmp(obj.Model,'DWNA') || strcmp(obj.Model,'NVEL') || strcmp(obj.Model,'DVEL') || strcmp(obj.Model,'IOSB')
                if strcmp(obj.Model,'DWNA')
                    Gamma=[0.5*obj.T^2 0; 0 0.5*obj.T^2; obj.T 0; 0 obj.T];
                end
                if strcmp(obj.Model,'NVEL') || strcmp(obj.Model,'DVEL')
                    Gamma=[0 0; 0 0;1 0;0 1];
                end
                if strcmp(obj.Model,'IOSB')
                    Gamma=[0 0; 0 0;0 0;0 1];
                end
                %syms Ts;
                %syms varx;
                %syms vary;
                %Gamma = [0.5*Ts^2 0; 0 0.5*Ts^2; Ts 0; 0 Ts]
                %var=[varx 0; 0 vary];
                %simplify(Gamma*var*Gamma')
                
                
                %Q=Gamma*Gamma';
                if obj.Dimensions==2
                    %Qx=Q*var(1);
                    %Qy=Q*var(2);
                    Q=Gamma*[var(1) 0; 0 var(2)]*Gamma';
                    %Q=[Qx(1,1) 0 Qx(1,2) 0; 0 Qy(1,1) 0 Qy(1,2);Qx(2,1) 0 Qx(2,2) 0;0 Qy(2,1) 0 Qy(2,2)];
                end
                if obj.Dimensions==3
                    Q=Gamma*[var(1) 0 0; 0 var(2) 0; 0 0 var(3)]*Gamma';
                    Q=[Q(1,1) 0 0 Q(1,2) 0 0; 0 Q(1,1) 0 0 Q(1,2) 0; 0 0 Q(1,1) 0 0 Q(1,2);Q(2,1) 0 0 Q(2,2) 0 0;0 Q(2,1) 0 0 Q(2,2) 0;0 0 Q(2,1) 0 0 Q(2,2)];
                end
            end
            % 3 degree
            if strcmp(obj.Model,'DWPA') || strcmp(obj.Model,'NACC')
                if strcmp(obj.Model,'NACC')
                    Gamma=[0; 0; 1];
                end
                Q=Gamma*Gamma';
                if obj.Dimensions==2
                    Qx=Q*var(1);
                    Qy=Q*var(2);
                    Q=[Qx(1,1) 0 Qx(1,2) 0 Qx(1,3) 0; 0 Qy(1,1) 0 Qy(1,2) 0 Qy(1,3);Qx(2,1) 0 Qx(2,2) 0 Qx(2,3) 0;0 Qy(2,1) 0 Qy(2,2) 0  Qy(2,3);Qx(3,1) 0 Qx(3,2) 0 Qx(3,3) 0;0 Qy(3,1) 0 Qy(3,2) 0 Qy(3,3)];
                end
                if obj.Dimensions==3
                    Q=[Q(1,1) 0 0 Q(1,2) 0 0; 0 Q(1,1) 0 0 Q(1,2) 0; 0 0 Q(1,1) 0 0 Q(1,2);Q(2,1) 0 0 Q(2,2) 0 0;0 Q(2,1) 0 0 Q(2,2) 0;0 0 Q(2,1) 0 0 Q(2,2)];
                end
            end
        end
        function [H]=getMeasurementMatrix(obj)
            if strcmp(obj.Model,'BVEL')
                H=[1 0 0 0 0 0;
                    0 1 0 0 0 0];
            end
            % 2 degree
            if strcmp(obj.Model,'DWNA') || strcmp(obj.Model,'NVEL') || strcmp(obj.Model,'DVEL')
                H=[1 0];
                if obj.Dimensions==2
                    H=[1 0 0 0; 0 1 0 0];
                end
                if obj.Dimensions==3
                    H=[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
                end
            end
            if strcmp(obj.Model,'DWNA') || strcmp(obj.Model,'NVEL') || strcmp(obj.Model,'DVEL')
                H=[1 0];
                if obj.Dimensions==2
                    H=[1 0 0 0; 0 1 0 0];
                end
                if obj.Dimensions==3
                    H=[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
                end
            end
            % 3 degree
            if strcmp(obj.Model,'IOSB')
                H=[1 0 0];
                if obj.Dimensions==2
                    H=[0 1 0 0];
                end
                if obj.Dimensions==3
                    H=1;
                end
            end
        end
        function [R]=getMeasurementNoiseMatrix(obj,var)
            % 2 degree
            if strcmp(obj.Model,'DWNA') || strcmp(obj.Model,'NVEL') || strcmp(obj.Model,'DVEL') || strcmp(obj.Model,'BVEL')
                R=var;
                if obj.Dimensions==2
                    R=eye(2)*var;
                end
                if obj.Dimensions==3
                    R=eye(3)*var;
                end
            end
            if strcmp(obj.Model,'DWNA') || strcmp(obj.Model,'NVEL') || strcmp(obj.Model,'DVEL')
                R=var;
                if obj.Dimensions==2
                    R=eye(2)*var;
                end
                if obj.Dimensions==3
                    R=eye(3)*var;
                end
            end
            % 3 degree
            if strcmp(obj.Model,'IOSB')
                R=var;
                if obj.Dimensions==2
                    R=eye(2)*var;
                end
                if obj.Dimensions==3
                    %R=eye(3)*var;
                end
            end
        end
    end
end

