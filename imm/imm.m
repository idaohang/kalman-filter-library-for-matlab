modelCV=CV(5);
F{1}=stateTransitionMatrixBuilder(modelCV.stateTransitionMatrix,modelCV.stateDescription,'s','s_x','v','v_x');
H{1}=[1 0];
modelCA=CA(5);
F{2}=stateTransitionMatrixBuilder(modelCA.stateTransitionMatrix,modelCA.stateDescription,'s','s_x','v','v_x','a','a_x');
H{2}=[1 0 0];

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
% Forced mode transitions
mstate(1:50) = 1;
mstate(51:70) = 2;
mstate(71:120) = 1;
mstate(121:150) = 2;
mstate(151:200) = 1;

% $$$ % Generate random mode transitions
mode(1) = 2;
for i = 2:nSteps
    r = rand;
    for j = size(p_ij,1)-1;
        if r < p_ij(mode(i-1),j);
            mode(i) = j;
        end
    end
    if mode(i) == 0
        mode(i) = size(p_ij,1);
    end
end
x(:,1)=[0;0;0];
for i = 2:nSteps
   xDim=size(F{mode(i)},2);
   x(:,i) = [F{mode(i)}*x(1:xDim,i-1) + randn;zeros(xDim<3)];
end

% Generate the measurements.
for i = 1:nSteps
    xDim=size(H{mode(i)},2);
    y(i)=[H{mode(i)} zeros(xDim<3)]*x(:,i)+randn;
end



stairs(mode);
ylim([0.5 2.5]);
set(gca,'YTickLabel',{'';'CV';'';'CA'})
title('Mode Probability');