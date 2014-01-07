classdef IMM < handle
    %IMM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        filters
        nFilters
        
        p_ji % transition probability
        u_i_k % mode probability
        u_i_k1 % mode probability predicted k1 denotes k-1
        u_ji % mixing weight
        L % model likelihood
        
        x
        P
    end
    
    methods
        function obj = IMM(varargin)
            obj.filters=varargin;
            obj.nFilters=length(varargin);
        end
        function setInitialModeProbability(obj,u_i)
            obj.u_i_k=u_i;
        end
        %% IMM functionality
        function predictModeProbability(obj)
            obj.u_i_k1=sum(obj.p_ji(:,1:end)*obj.u_i_k(:),2);
        end
        
        function mixingWeight(obj)
            for i=1:obj.nFilters
                for j=1:obj.nFilters
                    obj.u_ji(j,i)=(obj.p_ji(j,i)*obj.u_i_k(j))/obj.u_i_k1(i);
                end
            end
        end
        
        function mixingEstimate(obj)
            for i=1:obj.nFilters
                xDim=size(obj.filters{i}.F,2);
                for j=1:obj.nFilters
                    x(:,j)=sum(obj.filters{j}.x(1:xDim,:)*obj.u_ji(j,i));
                end
                obj.filters{i}.x=sum(x(:));
            end
        end
        %function mixingCovariance()
        function predictStates(obj)
            for i=1:obj.nFilters
                obj.filters{i}.predict(obj.filters{i}.x,0);
            end
        end
        %function predictCovariances();
        %function updateStates();
        %function updateCovariances();
        function calcModelLikelihood(obj)
            for i=1:obj.nFilters
                L(i)=obj.filters{i}.z-
            end
        end
        function updateModeProbability(obj)
            for j=1:obj.nFilters
                sum=obj.u_i_k1(j)*obj.L(j);
            end
            for i=1:obj.nFilters
                obj.u_i_k(i)=(obj.u_i_k1(i)*obj.L(i))/(sum);
            end
        end
        
        function overallEstimate(obj)
            for i=1:nFilters
                x(:,i)=obj.filters{i}.x*u_i_k(i);
            end
            obj.x=sum(x,2);
        end
        function overallCovariance(obj)
            obj.P=0;
        end
    end
    
end

