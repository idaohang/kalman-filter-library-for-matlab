clear all;
%% Model 1: CV-Model
modelCV=CV(5);
%% Model 2: CA-Model
modelCA=CA(5);
%% ground thruth
immgt=IMMGroundTruthGenerator(200,modelCV,modelCA);
% Forced mode transitions
immgt.mode(1:50) = 1;
immgt.mode(51:70) = 2;
immgt.mode(71:120) = 1;
immgt.mode(121:150) = 2;
immgt.mode(151:200) = 1;
immgt.initInitialState([0;0;0]);
immgt.generateGroundTruth();
measurementModels={[1 0],[1 0 0]};
immgt.generateMeasurements(measurementModels);
immgt.plotModeProbability();

kf1=KalmanFilter();