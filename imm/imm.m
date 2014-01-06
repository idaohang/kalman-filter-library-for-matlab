%% Model 1: CV-Model
modelCV=CV(5);
%% Model 2: CA-Model
modelCA=CA(5);
%% ground thruth
nSteps=1000;
mode(1)=1;
p_ij=[0.95 0.05;0.05 0.95];
%p_ij=[0.5 0.5;0.5 0.5];
for i=2:nSteps
    for j=1:size(p_ij,1)
        if rand < p_ij(mode(i-1),j)
            mode(i)=j;
        else
            mode(i)=mode(i-1);
        end
    end
end


immgt=IMMGroundTruthGenerator(modelCV,modelCA,200);
% Forced mode transitions
immgt.mode(1:50) = 1;
immgt.mode(51:70) = 2;
immgt.mode(71:120) = 1;
immgt.mode(121:150) = 2;
immgt.mode(151:200) = 1;

% Generate the measurements.
for i = 1:nSteps
    xDim=size(H{mode(i)},2);
    y(i)=[H{mode(i)} zeros(xDim<3)]*x(:,i)+randn;
end



stairs(mode);
ylim([0.5 2.5]);
set(gca,'YTickLabel',{'';'CV';'';'CA'})
title('Mode Probability');