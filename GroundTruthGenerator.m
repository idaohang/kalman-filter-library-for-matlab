classdef GroundTruthGenerator < handle
    %GROUNDTRUTHGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        steps % # discrete steps
        model
        initialState
        x % groundtruth
        measurements
    end
    
    methods
        function obj = GroundTruthGenerator(inModel,inSteps)
            obj.steps=inSteps;
            obj.model=inModel;
        end
        function initInitialState(obj,inX)
            obj.initialState=inX;
        end
        
        function generateGroundTruth(obj)
            obj.x=obj.initialState;
            for i=2:obj.steps
                obj.x(:,i)=obj.model.F*obj.x(:,i-1)+obj.model.Gamma*randn;
            end
        end
        
        function generateMeasurements(obj,inH,inR)
            for i=1:obj.steps
                obj.measurements(:,i)=inH*obj.x(:,i)+randn;
            end
        end
        
    end
    
end

