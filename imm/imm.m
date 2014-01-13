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
        
        
        
        mixingX
        mixingP
        
        overallX % estimate
        overallP
        
        measurements
        
        maxDoF
        
        % Data
        x % estimates
        P
        xM % mixing estimate
        PM
        filterX
        filterP
        modeProbability
        modelLikelihood
        S
        z
    end
    
    methods
        
        function obj = IMM(varargin)
            obj.filters=varargin;
            obj.nFilters=length(varargin);
            obj.maxDoF=0;
            for i=1:obj.nFilters
                obj.maxDoF=max(size(obj.filters{i}.F));
            end
        end
        
        function setInitialModeProbability(obj,u_i)
            obj.u_i_k=u_i;
        end
        
        %% IMM functionality
        function run(obj)
            for i=1:length(obj.measurements)
                predictModeProbability(obj);
                mixingWeight(obj);
                mixingEstimate(obj);
                mixingCovariance(obj);
                predictStatesAndCovariances(obj);
                updateStatesAndCovariances(obj,i);
                calcModelLikelihood(obj,i);
                updateModeProbability(obj);
                overallEstimate(obj);
                overallCovariance(obj);
                
                % save
                obj.x(:,i)=obj.overallX;
                obj.P{i}=obj.overallP;
                obj.modeProbability(:,i)=obj.u_i_k;
                obj.modelLikelihood(:,i)=obj.L;
                
                for j=1:obj.nFilters
                    obj.xM{i}{j}=obj.mixingX{j};
                    obj.PM{i}{j}=obj.mixingP{j};
                    obj.filterX{i}{j}=obj.filters{j}.x;
                    obj.filterP{i}{j}=obj.filters{j}.P;
                    obj.S(j,i)=obj.filters{j}.S;
                    obj.z(i)=obj.measurements(i)-obj.filters{j}.x(1);
                end
            end
        end
        
        function predictModeProbability(obj)
            obj.u_i_k1=obj.p_ji*obj.u_i_k;
        end
        
        function mixingWeight(obj)
            for i=1:obj.nFilters
                for j=1:obj.nFilters
                    obj.u_ji(j,i)=(obj.p_ji(i,j)*obj.u_i_k(j))/obj.u_i_k1(i);
                end
            end
        end
        
        function mixingEstimate(obj)
            for i=1:obj.nFilters
                for j=1:obj.nFilters
                    xDim=size(obj.filters{j}.F,2);
                    tempX(:,j)=[obj.filters{j}.x;zeros(obj.maxDoF-xDim,1)]*obj.u_ji(j,i);
                end
                obj.mixingX{i}=sum(tempX,2);
                tempX=[];
            end
        end
        
        function mixingCovariance(obj)
            for i=1:obj.nFilters
                tempP=0;
                for j=1:obj.nFilters
                    xDim=size(obj.filters{j}.P,2);
                    Pj=[obj.filters{j}.P zeros(xDim,obj.maxDoF-xDim);zeros(obj.maxDoF-xDim,xDim) zeros(obj.maxDoF-xDim,obj.maxDoF-xDim)];
                    xDimi=size(obj.filters{i}.P,2);
                    xi=([obj.filters{i}.x;zeros(obj.maxDoF-xDimi,1)]-obj.mixingX{j});
                    tempP=tempP+(Pj+xi*xi')*obj.u_ji(j,i);
                end
                obj.mixingP{i}=tempP;
            end
        end
        
        function predictStatesAndCovariances(obj)
            for i=1:obj.nFilters
                xDim=size(obj.filters{i}.F,2);
                tempX=obj.mixingX{i};
                tempP=obj.mixingP{i};
                obj.filters{i}.predict(tempX(1:xDim),tempP(1:xDim,1:xDim),0);
            end
        end
        
        function updateStatesAndCovariances(obj,j)
            for i=1:obj.nFilters
                obj.filters{i}.update(obj.measurements(j));
            end
        end
        
        function calcModelLikelihood(obj,i)
            for j=1:obj.nFilters
                obj.L(j)=gauss_pdf(obj.measurements(i)-obj.filters{j}.z,0,obj.filters{j}.S);
            end
        end
        
        function updateModeProbability(obj)
            sum=0;
            for j=1:obj.nFilters
                sum=sum+obj.u_i_k1(j)*obj.L(j);
            end
            for i=1:obj.nFilters
                obj.u_i_k(i)=(obj.u_i_k1(i)*obj.L(i))/(sum);
            end
        end
        
        function x = overallEstimate(obj)
            for i=1:obj.nFilters
                xDim=size(obj.filters{i}.F,2);
                x(:,i)=[obj.filters{i}.x;zeros(obj.maxDoF-xDim,1)]*obj.u_i_k(i);
            end
            obj.overallX=sum(x,2);
        end
        
        function overallCovariance(obj,x_mean)
            obj.overallP=0;
            for i=1:obj.nFilters
                xDim=size(obj.filters{i}.P,2);
                P=[obj.filters{i}.P zeros(xDim,obj.maxDoF-xDim);zeros(obj.maxDoF-xDim,xDim) zeros(obj.maxDoF-xDim,obj.maxDoF-xDim)];
                x=obj.overallX-[obj.filters{i}.x;zeros(obj.maxDoF-xDim,1)];
                obj.overallP=obj.overallP+(P+x*x')*obj.u_i_k(i);
            end
        end
        %% Plot Functions
        function plotEstimation(obj)
            for i=1:obj.maxDoF
                figure;
                plot(obj.x(i,:));
            end
        end
        
        function plotFilterEstimation(obj)
            for i=1:obj.nFilters
                figure;
                for j=1:length(obj.filterX)
                    tempX(:,j)=obj.filterX{i};
                end
                plot(obj.x(1,:));
            end
        end
        
                function tempX=getFilterEstimation(obj,n)
            %for i=1:obj.nFilters
                for j=1:length(obj.filterX)
                    tempX(:,j)=obj.filterX{j}{n};
                end
            %end
        end
        
        function plotCovariance(obj,i,j)
            for k=1:length(obj.P)
                p(k)=obj.P{k}(i,j);
            end
            plot(p);
        end
        
        function plotModeProbability(obj,i)
            plot(obj.modeProbability(i,:));
        end

        function plotModelLikelihood(obj,i)
            plot(obj.modelLikelihood(i,:));
        end
        
        function plotResidualCovariance(obj,i)
            plot(obj.S(i,:));
        end
    end
end

