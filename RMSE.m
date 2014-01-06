classdef RMSE<handle
    %RMSE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        trueState
        estimatedState
        RMSEresult
        iMonteCarlosRuns
        nMonteCarloRuns
    end
    
    methods
    %% Class Cunstructor
    function obj = RMSE()
            obj.iMonteCarlosRuns=0;
            obj.nMonteCarloRuns=0;
            obj.RMSEresult=0;
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
    %% calculateRMSE
    function [obj]=calculateRMSE(obj,n)
        for iMCRRun=1:100
            x_sch=obj.estimatedState{iMCRRun}-obj.trueState;
            obj.RMSEresult=obj.RMSEresult+x_sch(n,:).^2;
        end
        obj.RMSEresult=sqrt((1/100)*obj.RMSEresult);
    end
    end
    
end

