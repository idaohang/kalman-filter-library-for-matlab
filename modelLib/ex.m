%% Example Title
% Summary of example objective
model=CV(5);
model.stateDescription

F=stateTransitionMatrixBuilder(model.stateTransitionMatrix,model.stateDescription,'s','s_x','s','s_y','s','s_z','v','v_x','v','v_y','v','v_z');
F=stateTransitionMatrixBuilder(model.stateTransitionMatrix,model.stateDescription,'s','s_x','v','v_x','s','s_y','v','v_y','s','s_z','v','v_z');
%% Section 1 Title
% Description of first code block
a=1;

%% Section 2 Title
% Description of second code block
a=2;