classdef NEES<handle
    %NEES Summary of this class goes here
    %   Detailed explanation goes here
    % Version 1.1 - www.kalman-filter.de - www.kalman-filter.com
    properties
        iMonteCarlosRuns
        nMonteCarloRuns
        trueState
        estimatedState
        estimatedCovariance
        epsilon
        epsilonMean
        x_sch
    end
    
    methods
    %% Class Cunstructor
    function obj = NEES()
            obj.iMonteCarlosRuns=0;
            obj.nMonteCarloRuns=0;
      end
    %% setTrueState
    function [obj]=setTrueState(obj,state)
        obj.trueState=state;
    end
    %% addEstimatedState
    function [obj]=addEstimatedState(obj,state)
        obj.iMonteCarlosRuns=obj.iMonteCarlosRuns+1;
        obj.estimatedState{obj.iMonteCarlosRuns}=state;
    end
    %% addEstimatedCovariance
    function [obj]=addEstimatedCovariance(obj,covariance)
        obj.estimatedCovariance{obj.iMonteCarlosRuns}=covariance;
    end
    %% calculateNEES
    function [obj]=calculateNEES(obj)
        for iMCRRun=1:length(obj.estimatedState)
            obj.x_sch=obj.estimatedState{iMCRRun}-obj.trueState;
            for iStep=1:length(obj.estimatedState{1})
            obj.epsilon(iMCRRun,iStep)=obj.x_sch(:,iStep)'*pinv(obj.estimatedCovariance{iMCRRun}{iStep})*obj.x_sch(:,iStep);
            end
        end
        %cov(obj.x_sch(1,:),obj.x_sch(2,:)')
        for iStep=1:length(obj.estimatedState{1})
            obj.epsilonMean(iStep)=mean(obj.epsilon(:,iStep));
        end
    end
    %% plotNEES
    function [obj]=plotNEES(obj)
         degreeOfFredom=1;
plot(obj.epsilonMean);
hold on;
plot((chi2inv(0.975,degreeOfFredom*length(obj.estimatedState))/length(obj.estimatedState))*ones(length(obj.estimatedState{1}),1),'--');
hold on;
plot((chi2inv(0.025,degreeOfFredom*length(obj.estimatedState))/length(obj.estimatedState))*ones(length(obj.estimatedState{1}),1),'--');
     title(['Monte-Carlo-Runs: ' num2str(length(obj.estimatedState)) ' - Degree of Freedom: ' num2str(degreeOfFredom)]);
    end
        %% plotTrueCov
    function [obj]=plotTrueCov(obj)
       

    end
    end
    
end

