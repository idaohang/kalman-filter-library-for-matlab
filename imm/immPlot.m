classdef immPlot
    %IMMPLOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        
        function plotModeProbability(obj)
 
            stairs(immgt.mode);
            ylim([0.5 2.5]);
            set(gca,'YTickLabel',{'';'CV';'';'CA'})
            title('Mode Probability');
        end
        
    end
    
end

