classdef IMMGroundTruthGenerator < handle
    %IMMGROUNDTRUTHGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        steps % # discrete steps
        models % M={m(1),m(2),...m(n)}
        mode % e.g. {m_1(1),m_2(1),m_3(2)}
        x % groundtruth
        initialState
        measurements
    end
    
    methods
        function obj = IMMGroundTruthGenerator(inSteps,varargin)
            obj.steps=inSteps;
            obj.models=varargin;
        end
        function initInitialState(obj,inX)
            obj.initialState=inX;
        end
        function generateGroundTruth(obj)
            obj.x=obj.initialState;
            for i = 2:obj.steps
                xDim=size(obj.models{obj.mode(i)}.F,2);
                obj.x(:,i) = [obj.models{obj.mode(i)}.F*obj.x(1:xDim,i-1) + randn;zeros(xDim<3)];
            end
        end
        function generateMeasurements(obj,inH)
            for i=1:obj.steps
                xDim=size(inH{obj.mode(i)},2);
                obj.measurements(:,i)=[inH{obj.mode(i)} zeros(xDim<3)]*obj.x(:,i)+randn;
            end
        end
        function plotModeProbability(obj)
            stairs(obj.mode);
            ylim([0.5 2.5]);
            set(gca,'YTickLabel',{'';'CV';'';'CA'})
            title('Mode Probability');
        end
    end
end