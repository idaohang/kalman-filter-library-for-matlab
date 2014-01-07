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
%immgt.plotModeProbability();


%% Setup Kalman Filter 1
kf1=KalmanFilter(modelCV);
x0=[0;0];
% configure start values
kf1.initInitialState(x0);
kf1.initInitialCovariance(eye(2));
% configure measurement model
kf1.setMeasurmentModel('s_x');
kf1.setMeasurmentCovariance(eye(2));
%% Setup Kalman Filter 2
kf2=KalmanFilter(modelCA);
x0=[0;0;0];
% configure start values
kf2.initInitialState(x0);
kf2.initInitialCovariance(eye(3));
% configure measurement model
kf2.setMeasurmentModel('s_x');
kf2.setMeasurmentCovariance(eye(3));

%% Setup IMM
imm=IMM(kf1,kf2);
imm.p_ji=[0.95 0.05;0.05 0.95];
imm.setInitialModeProbability([0.9;0.1])
imm.predictModeProbability();
imm.mixingWeight();
%imm.mixingCovariances();
imm.predictStates();
%imm.predictCovariances();
%imm.updateStates();
%imm.updateCovariances();
%imm.updateModeProbability();
%imm.overallEstimate();
%imm.overallCovariance();