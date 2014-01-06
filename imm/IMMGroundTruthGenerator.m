classdef IMMGroundTruthGenerator
    %IMMGROUNDTRUTHGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        steps % # discrete steps
        models % M={m(1),m(2),...m(n)}
        mode % e.g. {m_1(1),m_2(1),m_3(2)}
        x % groundtruth
        measurements
    end
    
    methods
        function obj = IMMGroundTruthGenerator(inSteps,varargin)
            obj.steps=inSteps;
            obj.models=varargin;
        end
        function generateGroundTruth(obj)
            for i = 2:obj.steps
                xDim=size(obj.models{obj.mode(i)}.F,2);
                obj.x(:,i) = [obj.models{obj.mode(i)}.F*obj.x(1:xDim,i-1) + randn;zeros(xDim<3)];
            end
        end
    end
    
end

