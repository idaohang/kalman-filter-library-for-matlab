function [ outF, outFDes ] = stateTransitionMatrixBuilder(inF, inDes, varargin)
%STATETRANSITIONMATRIXBUILDER Summary of this function goes here
%   Detailed explanation goes here
varargin=varargin{:};
nVarargs = length(varargin);
nVarInDes = length(inDes);
outF=NaN(nVarargs/2,nVarargs/2);
for i=1:nVarargs/2
    for j=1:nVarargs/2
        if strcmp(getPostFix(varargin{(i-1)+(i+1)}),getPostFix(varargin{(j-1)+(j+1)}))
            n=strfind(inDes',varargin{(i-1)+i});
            k=strfind(inDes',varargin{(j-1)+j});
            outF(i,j)=inF(n,k);
        else % uncoupled States
            outF(i,j)=0;
        end
    end
end
end

function postfix = getPostFix(inString)
postfix=inString(2:end);
end

