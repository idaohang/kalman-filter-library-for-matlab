classdef IMM < handle
    %IMM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        p_ij % transitionProbability
    end
    
    methods
        function u_i_k = predictModeProbability(u_j_k)
            u_i_k=sum(p_ij()*u_j_k(j));
        end
    end
    
end

