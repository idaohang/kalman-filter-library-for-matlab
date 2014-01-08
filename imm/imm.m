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
        
        overallX
        overallP
        
        measurments
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
        function run(obj)
            for i=1:length(obj.measurments)
                predictModeProbability(obj);
                mixingWeight(obj);
                mixingEstimate(obj);
                mixingCovariance(obj);
                predictStatesAndCovariances(obj);
                updateStatesAndCovariances(obj,i);
                calcModelLikelihood(obj);
                updateModeProbability(obj);
                overallEstimate(obj);
                overallCovariance(obj);
            end
        end
        
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
                    x(:,j)=[obj.filters{j}.x(1:xDim,:);zeros(xDim-size(obj.filters{j}.x,1),1)]*obj.u_ji(j,i);
                end
                obj.x{i}=sum(x,2);
                x=[];
            end
        end
        
        function mixingCovariance(obj)
            for i=1:obj.nFilters
                P=0;
                for j=1:obj.nFilters
                    P=P+(obj.filters{j}.P+(obj.filters{i}.x-obj.x{j})*(obj.filters{i}.x-obj.x{j})')*obj.u_ji(j,i);
                end
                obj.P{i}=P;
            end
            
        end
        
        function predictStatesAndCovariances(obj)
            for i=1:obj.nFilters
                obj.filters{i}.predict(obj.x{i},obj.P{i},0);
            end
        end
        
        function updateStatesAndCovariances(obj,j)
            for i=1:obj.nFilters
                obj.filters{i}.update(obj.measurement(j),0);
            end
        end
        
        function calcModelLikelihood(obj,measurment)
            for i=1:obj.nFilters
                obj.L(i)=normpdf(measurment-obj.filters{i}.z,0,obj.filters{i}.S);
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
            for i=1:obj.nFilters
                x(:,i)=obj.filters{i}.x*obj.u_i_k(i);
            end
            obj.overallX=sum(x,2);
        end
        
        function overallCovariance(obj)
            obj.overallP=0;
            for i=1:obj.nFilters
                obj.overallP=obj.overallP+(obj.filters{i}.P+(obj.overallX-obj.filters{i}.x)*(obj.overallX-obj.filters{i}.x)')*obj.u_i_k(i);
            end
        end
    end
end

