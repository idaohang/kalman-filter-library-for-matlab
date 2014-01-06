function [ outGamma] = processNoiseVectorBuilder(inGamma, inDes, varargin)
%STATETRANSITIONMATRIXBUILDER Summary of this function goes here
%   Detailed explanation goes here
varargin=varargin{:};
nVarargs = length(varargin);
nVarInDes = length(inDes);
outGamma=NaN(nVarargs/2,1);
for i=1:nVarargs/2
    for j=1:nVarInDes
        if strcmp(inDes(j),getPreFix(varargin{(i-1)+(i+1)}))
             outGamma(i)=inGamma(j);
        end
    end
end
end

function prefix = getPreFix(inString)
prefix=inString(1);
end

